from __future__ import print_function

from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, HomePosition, State
from mavros_msgs.srv import CommandBool, WaypointClear, WaypointPush, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix, Imu

import math
import time

import rospy
from pyquaternion import Quaternion
from haversine import haversine, Unit


class Drone:
    def __init__(self):
        self.state = None
        self.altitude = None
        self.global_pos = None
        self.global_target = None
        self.imu_data = None
        self.yaw = None
        self.pitch = None
        self.roll = None
        self.home_pos = None
        self.local_pos = None
        self.local_target = None

        self.running = False
        self.update_fast = 10 # Hz
        self.update_slow = 1 # Hz

        self.sub_topics_ready = {
            key: False
            for key in ['alt', 'global_pos', 'home_pos', 'local_pos', 'mission_wp', 'state', 'imu']
        }

        # ROS services
        service_timeout = 30
        try:
            _pprint("Waiting for ROS services...")
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('/mavros/set_mode', service_timeout)
            rospy.wait_for_service('/mavros/cmd/land', service_timeout)
        except:
            _pprint("Failed to connect to ROS services (are they running?)")
            exit(0)

        self.arm_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
        self.mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff_srv = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
        self.landing_srv = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)

        # ROS publishers
        self.local_target_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.global_target_pub = rospy.Publisher('mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)

        # ROS subscribers
        self.state_sub = rospy.Subscriber('mavros/state', State, self._state_callback)
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude, self._altitude_callback)
        self.gps_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, self._gps_callback)
        self.imu_data_sub = rospy.Subscriber('mavros/imu/data', Imu, self._imu_data_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home', HomePosition, self._home_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self._local_position_callback)

    def start(self):
        if not self.running:
            self.running = True

            _pprint("Entering GUIDED mode")
            self.set_mode('GUIDED')
            rospy.init_node('guided_node')

    '''
    Callbacks
    '''

    def _state_callback(self, data):
        self.state = data

    def _altitude_callback(self, data):
        self.altitude = data
        print(data)

        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def _gps_callback(self, data):
        self.global_pos = data

        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True

    def _imu_data_callback(self, data):
        self.imu_data = data
        o = data.orientation
        q = Quaternion(o.w, o.x, o.y, o.z)
        self.yaw, self.pitch, self.roll = q.yaw_pitch_roll

        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True

    def _home_position_callback(self, data):
        self.home_pos = data

        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True

    def _local_position_callback(self, data):
        self.local_pos = data

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def _mission_wp_callback(self, data):
        self.mission_wp = data

        if not self.sub_topics_ready['mission_wp']:
            self.sub_topics_ready['mission_wp'] = True

    '''
    Control Methods
    '''

    def arm(self, timeout=60):
        if self.running:
            start_time = time.time()
            rate = rospy.Rate(self.update_slow)
            while not self.state.armed and time.time() < start_time + timeout:
                try:
                    self.arm_srv(True)
                except:
                    return False
                rate.sleep()
            if self.state.armed:
                return True
        return False

    def disarm(self):
        if self.running:
            return self.arm_srv(False)

    def set_mode(self, mode):
        return self.mode_srv(custom_mode=mode)

    def takeoff(self, height, timeout=60):
        if self.running:
            if not self.state.armed:
                self.arm(timeout=timeout)
            try:
                self.takeoff_srv(latitude=self.global_pos.latitude, longitude=self.global_pos.longitude, altitude=height)
            except:
                return False
            return True
        return False

    def land_here(self):
        self.land(self.global_pos.latitude, self.global_pos.longitude)

    def land(self, lat, lng):
        if self.running:
            self.landing_srv(latitude=lat, longitude=lng)

    def goto_localpos(self, x, y, z, relative=False):
        if relative:
            pos = self.local_pos.pose.position
            target = local_position_target(pos.x + x, pos.y + y, pos.z + z)
        else:
            target = local_position_target(x, y, z)
        self.local_target = target
        self.local_target_pub.publish(target)

    def reached_localpos(self, err=0.3):
        if self.local_target is not None:
            tpos = self.local_target.pose.position
            cpos = self.local_pos.pose.position
            dist = _3ddist(tpos.x, tpos.y, tpos.z, cpos.x, cpos.y, cpos.z)
            if dist <= err:
                return True, dist
            else:
                return False, dist
        return True, 0.0

    def goto_globalpos(self, latitude, longitude, altitude=None):
        if altitude is None:
            target = global_position_target(latitude, longitude, self.global_pos.altitude)
        else:
            target = global_position_target(latitude, longitude, altitude)
        self.global_target = target
        self.global_target_pub.publish(target)

    def reached_globalpos(self, err=0.3):
        if self.global_target is not None:
            sdist = haversine((self.global_pos.latitude, self.global_pos.longitude), (self.global_target.pose.position.latitude, self.global_target.pose.position.longitude), unit=Unit.METERS)
            dist = math.sqrt(pow(sdist, 2) + pow(self.global_pos.altitude - self.global_target.pose.position.altitude, 2))
            print(self.global_pos.altitude, self.global_target.pose.position.altitude)
            if dist <= err:
                return True, dist
            else:
                return False, dist
        return True, 0.0


def local_position_target(x, y, z):
    target = PoseStamped()
    target.header.stamp = rospy.Time.now()

    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.position.z = z

    return target


def global_position_target(latitude, longitude, altitude):
    target = GeoPoseStamped()
    target.header.stamp = rospy.Time.now()

    target.pose.position.latitude = latitude
    target.pose.position.longitude = longitude
    target.pose.position.altitude = altitude

    return target


def _3ddist(x1, y1, z1, x2, y2, z2):
    return math.sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2))


def _pprint(msg, end='\n'):
    print("[CTRL] " + msg, end=end)
