import sys
from telnetlib import EC
import time
import logging
import argparse
import concurrent.futures

from collections import deque
from unicodedata import name
from readerwriterlock import rwlock

sys.path.append(r'/home/bhagatsj/RobotSystems/lib')
from picarx import Picarx
from utils import reset_mcu
from picamera import PiCamera
from picamera.array import PiRGBArray
from w3_sensors import *
from rossros import *

reset_mcu()

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.ERROR , datefmt="%H:%M:%S ")

def main(config):
    #Line following simultaneity
    if config.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    #Instantiate object to talk to different components of the system
    car = Picarx() #the car class already has built in sensing capabilities, so I am reusing it.
    interpretor = Grayscale_Interpreter(75, 1.0)
    controller = Controller(car, scale=0.9)

    #Set up the buses
    sensor_bus = Bus(name="SensorBus")
    interpretor_bus = Bus(name="InterpretorBus")
    termination_bus = Bus(name="TerminationBus")

    #Spin up consumer, producers and the timer
    timer = Timer(termination_bus, duration=config.time, name="timer")
    sensor_producer = Producer(
        car.get_adc_value, 
        sensor_bus, 
        delay=0.1,
        termination_busses=termination_bus)
    interpret_cp = ConsumerProducer(
        interpretor.edge_detect, 
        sensor_bus, 
        interpretor_bus, 
        delay=0.1,
        termination_busses=termination_bus)
    controller_consumer = Consumer(
        controller.set_angle, 
        interpretor_bus, 
        delay=0.1,
        termination_busses=termination_bus)
    
    #Follow the line for n seconds
    controller.start_car()
    runConcurrently([timer, sensor_producer, interpret_cp, controller_consumer])
    controller.stop_car()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--time', default=10,
                        help='Duration of running the program')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Debug flag')
    main(parser.parse_args())
