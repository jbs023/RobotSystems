import sys
import math
import time
import logging
import argparse
import numpy as np
from lane_detection import detect_lane

sys.path.append(r'/home/bhagatsj/RobotSystems/lib')
from picarx import Picarx
from utils import reset_mcu
from picamera import PiCamera
from picamera.array import PiRGBArray

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
    def __init__(self, car, sensor, camera, scale=1.0):
        self.car = car
        self.sensor = sensor
        self.angle = 5
        self.scale = scale
        self.camera = camera

    def get_turn_angle(self):
        #Calculate turn angle
        adc_list = self.car.get_adc_value()
        maneuver_val = self.sensor.get_manuever_val(adc_list)
        angle = self.scale*self.angle*maneuver_val

        return angle

    def follow_line(self, duration):
        """Follow the line using grey scale camera"""
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

    def follow_line_cv(self, duration):
        """Follow the line using computer vision"""
        rel_time = 0
        self.camera.resolution = (640,480)
        self.camera.framerate = 24
        rawCapture = PiRGBArray(self.camera, size=self.camera.resolution)  

        self.car.forward(30)
        start_time = time.time()
        for frame in self.camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
            #Repurpose lane lines to simply follow a line
            height, width, _ = frame.shap
            lane_lines = detect_lane(frame)
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
            y_offset = int(height / 2)

            angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
            angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
            steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

            logging.debug('new steering angle: %s' % steering_angle)
            self.car.set_dir_servo_angle(steering_angle)
            time.sleep(0.5)

            rel_time = time.time() - start_time
            if rel_time >= duration:
                break
        self.car.stop()



def main(config):
    if config.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    car = Picarx()
    sensor = Grayscale_Interpreter(75, 1.0)
    camera = PiCamera()
    controller = Controller(car, sensor, camera, scale=0.9)
    controller.follow_line(config.time)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--time', default=10,
                        help='Duration of running the program')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Debug flag')
    main(parser.parse_args())
