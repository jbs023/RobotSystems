import sys
import math
import time
import logging
import numpy as np
# from lane_detection import detect_lane

sys.path.append(r'../lib')
from picarx import Picarx
from utils import reset_mcu
# from picamera import PiCamera
# from picamera.array import PiRGBArray

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

    def __init__(self, sensitivity=0, polarity=-1):
        self.sens = sensitivity
        self.pol = polarity

    def edge_detect(self, adc_list):
        """
        Function to process the grayscale data
        :param adc_list: The array of grayscale data from the car
        """
        # Normalize the array to the maximum value obtained
        gry_list_norm = [float(i)/max(adc_list) for i in adc_list]
        gry_list_diff = max(gry_list_norm)-min(gry_list_norm)
        logging.debug("Norm ADC List: {}".format(gry_list_norm))
        logging.debug("Grey Diff: {}".format(gry_list_diff))

        # If the difference is larger than the tolerance, try to detect an edge
        if gry_list_diff > self.sens:
            rel_dir = gry_list_norm[0]-gry_list_norm[2]

            # Calculate the amount of error to make a more continuous relative
            # direction. The deviation of the max or min value from the avg is
            # determined to be the error.
            if self.pol == 1:
                error = (max(gry_list_norm)-np.mean(gry_list_norm))*(2/3)
            elif self.pol == -1:
                error = (min(gry_list_norm)-np.mean(gry_list_norm))*(2/3)
                
            # The relative direction is then multiplied by error and polarity
            # to make a distinction between "just off-centered" and "very off-
            # centered"
            rel_dir_pol = rel_dir*error*self.pol
        else:
            rel_dir_pol = 0

        logging.debug("Rel Dir: {}".format(rel_dir_pol))
        return rel_dir_pol


class Controller():
    '''Class that controls the Picarx'''
    def __init__(self, car, scale=40):
        self.car = car
        self.scale = scale

        self.car.set_camera_servo2_angle(-25)

    def start_car(self):
        #Calculate turn angle
        self.car.forward(40)

    def stop_car(self):
        self.car.forward(0)
        # TODO: This isn't working for some reason?
        # self.car.stop()

    def set_angle(self, rel_dir):
        """Follow the line using grey scale camera"""
        # print("-1 * {} * {}".format(rel_dir, self.scale))
        self.car.set_dir_servo_angle(-1*rel_dir*self.scale)

    #FIXME: I never got this working properly
    # def follow_line_cv(self, duration):
    #     """Follow the line using computer vision"""
    #     camera = PiCamera()
    #     time.sleep(2) #let the camera warm up
        
    #     rel_time = 0
    #     camera.resolution= (640,480)
    #     camera.framerate = 24
    #     rawCapture = PiRGBArray(camera, size=camera.resolution)  

    #     self.car.forward(30)
    #     start_time = time.time()
    #     for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #         #Repurpose lane lines to simply follow a line
    #         img = frame.array
    #         height, width, _ = img.shape
    #         lane_lines = detect_lane(img)

    #         logging.debug("Lane Lines: {}".format(lane_lines))
    #         if len(lane_lines) > 0:
    #             x1, _, x2, _ = lane_lines[0][0]
    #             x_offset = x2 - x1
    #             y_offset = int(height / 2)

                
    #             logging.debug("X_off, Y_off: ({}, {})".format(x_offset, y_offset))
    #             angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    #             angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    #             steering_angle = angle_to_mid_deg  # this is the steering angle needed by picar front wheel

    #             logging.debug('new steering angle: %s' % steering_angle)
    #             self.car.set_dir_servo_angle(steering_angle)
    #         else:
    #             self.car.set_dir_servo_angle(0)
            
    #         time.sleep(0.01)
    #         rawCapture.truncate(0)
    #         rel_time = time.time() - start_time
    #         if rel_time >= duration:
    #             break
    #     self.car.stop()
