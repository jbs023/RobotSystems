import sys
import logging
import argparse

sys.path.append(r'../lib')
from picarx import Picarx
from utils import reset_mcu
from sensors import GrayscaleSensor, UltraSonicSensor
from interpretors import Grayscale_Interpreter, UltraSonic_Interpreter
from controller import Controller
from rossros import *

reset_mcu()

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.ERROR , datefmt="%H:%M:%S ")

def main(config):
    #Line following simultaneity
    if config.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    #Instantiate object to talk to different components of the system
    gs_sensor = GrayscaleSensor()
    gs_inter = Grayscale_Interpreter()
    us_sensor = UltraSonicSensor()
    us_inter = UltraSonic_Interpreter()
    controller = Controller()

    #Let everything warm up
    time.sleep(2)

    #Set up the buses
    gs_sens_bus = Bus(name="GSSensor")
    gs_inter_bus = Bus(name="GSInterpretor")
    us_sens_bus = Bus(name="USSensor")
    us_inter_bus = Bus(name="USInterpretor")
    termination_bus = Bus(False, name="TerminationBus")

    #Set up timer
    timer = Timer(termination_bus, duration=float(config.time), termination_busses=termination_bus, name="timer")

    #Consumer producers for line following
    gs_sens_prod = Producer(
        gs_sensor.read, 
        gs_sens_bus, 
        delay=0.00,
        termination_busses=termination_bus,
        name="Sensor")
    gs_inter_cp = ConsumerProducer(
        gs_inter.get_rel_dir, 
        gs_sens_bus, 
        gs_inter_bus, 
        delay=0.00,
        termination_busses=termination_bus,
        name="Interpretor")
    gs_control_cons = Consumer(
        controller.set_angle, 
        gs_inter_bus, 
        delay=0.01,
        termination_busses=termination_bus,
        name="Controller")

    #Consumer producers for emergency stopping
    us_sens_prod = Producer(
        us_sensor.read, 
        us_sens_bus, 
        delay=0.01,
        termination_busses=termination_bus,
        name="Sensor")
    us_inter_cp = ConsumerProducer(
        us_inter.interpret, 
        us_sens_bus, 
        us_inter_bus,
        delay=0.01,
        termination_busses=termination_bus,
        name="Sensor")
    us_control_cons = Consumer(
        controller.emergency_stop, 
        us_inter_bus, 
        delay=0.01,
        termination_busses=termination_bus,
        name="Controller")
    
    #Follow the line for n seconds
    try:
        controller.start_car()
        runConcurrently([timer, us_sens_prod, us_inter_cp, us_control_cons,
                                gs_sens_prod, gs_inter_cp, gs_control_cons])
        controller.stop_car()
    except Exception as e:
        controller.stop_car()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--time', default=3,
                        help='Duration of running the program')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Debug flag')
    main(parser.parse_args())