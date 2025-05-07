from onedr.drone_controller import DroneController
import logging


def main(args=None):

    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    drone_controller  = DroneController(logging.getLogger())
    mydrone = drone_controller.connect('udp:127.0.0.1:14550')
    print(mydrone)
     

if __name__ == '__main__':
    print("OK")
    main()