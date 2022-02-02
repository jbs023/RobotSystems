import sys
import logging
import argparse

sys.path.append(r'../lib')
from picarx import Picarx
from utils import reset_mcu
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
    interpretor = Grayscale_Interpreter()
    controller = Controller(car)

    #Let everything warm up
    time.sleep(2)

    #Set up the buses
    sensor_bus = Bus(name="SensorBus")
    interpretor_bus = Bus(name="InterpretorBus")
    termination_bus = Bus(False, name="TerminationBus")

    #Spin up consumer, producers and the timer
    timer = Timer(termination_bus, duration=float(config.time), termination_busses=termination_bus, name="timer")
    sensor_producer = Producer(
        car.get_adc_value, 
        sensor_bus, 
        delay=0.01,
        termination_busses=termination_bus,
        name="Sensor")
    interpret_cp = ConsumerProducer(
        interpretor.edge_detect, 
        sensor_bus, 
        interpretor_bus, 
        delay=0.01,
        termination_busses=termination_bus,
        name="Interpretor")
    # controller_consumer = Consumer(
    #     controller.set_angle, 
    #     interpretor_bus, 
    #     delay=0.02,
    #     termination_busses=termination_bus,
    #     name="Controller")
    
    #Follow the line for n seconds
    # controller.start_car()
    runConcurrently([timer, sensor_producer, interpret_cp])
    # controller.stop_car()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--time', default=1,
                        help='Duration of running the program')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Debug flag')
    main(parser.parse_args())
