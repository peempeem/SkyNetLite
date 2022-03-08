import droneController
import time

def main():
    drone = droneController.Drone()
    drone.start()
    drone.takeoff(5)
    time.sleep(5)
    lat = -35.3632621
    lon = 149.1652374

    drone.goto_globalpos(lat + 0.0001, lon + 0.00015)
    time.sleep(10)

    drone.goto_globalpos(lat-0.0002, lon)
    time.sleep(10)


    '''drone.goto_localpos(5, 5, 10)
    wait_until_reached(drone, True)

    drone.goto_localpos(-5, -10, 0, relative=True)
    wait_until_reached(drone, True)

    drone.goto_localpos(0, 0, 5)
    wait_until_reached(drone, True)'''

    drone.land_here()

def wait_until_reached(drone, local):
    reached = False
    while not reached:
        if local:
            reached, dist = drone.reached_localpos()
        else:
            reached, dist = drone.reached_globalpos()
        print(dist)
        time.sleep(0.1)
    print('reached')

if __name__ == '__main__':
    main()
