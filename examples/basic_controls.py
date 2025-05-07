from onedr.drone_controller import DroneController
import logging
import time


def main(args=None):

    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    drone_controller  = DroneController(logging.getLogger())
    mydrone = drone_controller.connect('udp:127.0.0.1:14550')
    mydrone.set_guided()
    time.sleep(5)
    mydrone.arm(wait_for_position=True)
    time.sleep(5)
    mydrone.take_off()
 

if __name__ == '__main__':
    print("OK")
    main()