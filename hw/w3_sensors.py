import argparse
import sys
import time
import logging

sys.path.append(r'/home/bhagatsj/RobotSystems/lib')
from picarx import Picarx
from utils import reset_mcu
reset_mcu()


logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.ERROR , datefmt="%H:%M:%S ")

class Grayscale_Interpreter():
    """
    Class to interpret the values from the greyscale cameras
    ...

    Attributes
    ----------
    sensitivity : int
        the expected difference between the line and the rest of the floor
    polarity : [1,-1]
        Direction of the sensitivity. Positive will look for a darker 
        line and negative will look for a lighter line
    """

    def __init__(self, sensitivity, polarity=1):
        self.sens = sensitivity
        self.pol = polarity

        assert((self.pol == 1) or (self.pol == -1))

    def get_line_status(self, adc_list): 
        #If on the line these should be large, otherwise they'll be small         
        center_diff1 = adc_list[1]-adc_list[0]
        center_diff2 = adc_list[1]-adc_list[2]
        edge_diff = adc_list[0]-adc_list[2]
        if self.pol <= 0:
            #If pol < 0, we want to look for a lighter line
            center_diff1 *= -1
            center_diff2 *= 2

        #Get relative direction of th eline
        line_direction = "straight"
        if center_diff1 < self.sens:
            line_direction = 'left'
        elif center_diff2 < self.sens:
            line_direction = 'right'

        logging.debug("Center Diff 1: {}".format(center_diff1))
        logging.debug("Center Diff 2: {}".format(center_diff2))
        logging.debug("Line Direction: {}".format(line_direction))
        return line_direction

    def get_manuever_val(self, adc_list):
        line_direction = self.get_line_status(adc_list)

        maneuver_val = 0
        if line_direction == "left":
            maneuver_val = -1
        elif line_direction == "right":
            maneuver_val = 1

        logging.debug("Manuever Val: {}".format(maneuver_val))
        return maneuver_val

class Controller():
    '''Class that controls the Picarx'''
    def __init__(self, car, sensor, scale=1.0):
        self.car = car
        self.sensor = sensor
        self.angle = 5
        self.scale = scale

    def get_turn_angle(self):
        #Calculate turn angle
        adc_list = self.car.get_adc_value()
        maneuver_val = self.sensor.get_manuever_val(adc_list)
        angle = self.scale*self.angle*maneuver_val

        return angle

    def follow_line(self, duration):
        #Start on line and follow it.
        start_time = time.time()
        rel_time = 0
        prev_angle = 0

        self.car.forward(30)
        while rel_time < duration:
            angle = self.get_turn_angle()
            if angle != prev_angle:
                self.car.set_dir_servo_angle(angle)
            
            prev_angle = angle
            time.sleep(0.5)
            rel_time = time.time() - start_time
        self.car.stop()


def main(config):
    if config.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    car = Picarx()
    sensor = Grayscale_Interpreter(75, 1.0)
    controller = Controller(car, sensor, scale=0.9)
    controller.follow_line(config.time)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--time', default=10,
                        help='Duration of running the program')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Debug flag')
    main(parser.parse_args())
