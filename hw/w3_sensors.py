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

    def concurrent_get_m_value(self, sensor_bus, interpretor_bus, delay_time):
        '''Function to support conncurrent sensor interpretation'''
        while True:
            adc_list = sensor_bus.read()
            maneuver_val = self.get_manuever_val(adc_list)
            interpretor_bus.write(maneuver_val)
            time.sleep(delay_time)


class Controller():
    '''Class that controls the Picarx'''
    def __init__(self, car, scale=1.0):
        self.car = car
        self.angle = 5
        self.scale = scale

        self.car.set_camera_servo2_angle(-25)

    def start_car(self):
        #Calculate turn angle
        self.car.forward(40)

    def stop_car(self):
        self.car.stop()

    def set_angle(self, maneuver_val):
        """Follow the line using grey scale camera"""
        angle = self.scale*self.angle*maneuver_val
        self.car.set_dir_servo_angle(angle)

    def concurrent_set_angle(self, interpretor_bus, controller, delay_time):
        '''Function to support concurrent controller updates'''
        while True:
            maneuver_val = interpretor_bus.read()
            maneuver_val = controller.set_angle(maneuver_val)
            time.sleep(delay_time)

    def follow_line(self, duration, maneuver_val):
        """Follow the line using grey scale camera"""
        start_time = time.time()
        rel_time = 0
        prev_angle = 0
 
        while rel_time < duration:
            angle = self.scale*self.angle*maneuver_val
            if angle != prev_angle:
                self.car.set_dir_servo_angle(angle)
            
            prev_angle = angle
            time.sleep(0.5)
            rel_time = time.time() - start_time

    #FIXME: This is definitely still a WIP, it doesn't properly work yet.
    def follow_line_cv(self, duration):
        """Follow the line using computer vision"""
        camera = PiCamera()
        time.sleep(2) #let the camera warm up
        
        rel_time = 0
        camera.resolution= (640,480)
        camera.framerate = 24
        rawCapture = PiRGBArray(camera, size=camera.resolution)  

        self.car.forward(30)
        start_time = time.time()
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            #Repurpose lane lines to simply follow a line
            img = frame.array
            height, width, _ = img.shape
            lane_lines = detect_lane(img)

            logging.debug("Lane Lines: {}".format(lane_lines))
            if len(lane_lines) > 0:
                x1, _, x2, _ = lane_lines[0][0]
                x_offset = x2 - x1
                y_offset = int(height / 2)

                
                logging.debug("X_off, Y_off: ({}, {})".format(x_offset, y_offset))
                angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
                angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
                steering_angle = angle_to_mid_deg  # this is the steering angle needed by picar front wheel

                logging.debug('new steering angle: %s' % steering_angle)
                self.car.set_dir_servo_angle(steering_angle)
            else:
                self.car.set_dir_servo_angle(0)
            
            time.sleep(0.01)
            rawCapture.truncate(0)
            rel_time = time.time() - start_time
            if rel_time >= duration:
                break
        self.car.stop()