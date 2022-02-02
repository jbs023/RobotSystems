import sys
from telnetlib import EC
import time
import logging
import argparse
import concurrent.futures

from collections import deque
from readerwriterlock import rwlock

sys.path.append(r'/home/bhagatsj/RobotSystems/lib')
from picarx import Picarx
from utils import reset_mcu
from picamera import PiCamera
from picamera.array import PiRGBArray
from w3_sensors import *

reset_mcu()

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.ERROR , datefmt="%H:%M:%S ")

class Bus():
    def __init__(self):
        self.lock = rwlock.RWLockWriteD()
        self.message = None

    def read(self):
        with self.lock.gen_rlock():
            msg = self.message
        return self.message

    def write(self, message):
        with self.lock.gen_wlock():
            self.message = message

def main(config):
    #Line following simultaneity
    if config.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    car = Picarx()
    interpretor = Grayscale_Interpreter(75, 1.0)
    controller = Controller(car, scale=0.9)
    sensor_bus = Bus()
    interpretor_bus = Bus()

    duration = 30 #run the program for 30 seconds
    controller.start_car()
    
    #Spin up futures to write to the busses
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        eSensor = executor.submit(car.conncurent_adc_values, sensor_bus, car, 0.1)
        eInterpreter = executor.submit(interpretor.concurrent_get_m_value, sensor_bus, interpretor_bus, interpretor, 0.1)
        eController = executor.submit(controller.concurrent_set_angle, interpretor_bus, controller, 0.1)

        #TODO: Is this necessary?
        eSensor.result()
        eInterpreter.result()
        eController.result()

        #Use main thread for the controller to follow line
        start_time = time.time()
        rel_time = 0
        while rel_time < duration:
            time.sleep(1)
            rel_time = time.time() - start_time
                    
    controller.stop_car()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--time', default=10,
                        help='Duration of running the program')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Debug flag')
    main(parser.parse_args())
