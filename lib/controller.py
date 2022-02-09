import sys
import math
import time
import logging
from lane_detection import detect_lane

sys.path.append(r'../lib')
from picarx import Picarx
from utils import reset_mcu
from picamera import PiCamera
from picamera.array import PiRGBArray

reset_mcu()

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.ERROR , datefmt="%H:%M:%S ")

class Controller():
    '''Wrapper class around the PiCarx class'''
    def __init__(self, scale=40):
        self.car = Picarx()
        self.scale = scale
        self.emergency = False

        self.car.set_camera_servo2_angle(-25)

    def start_car(self):
        #Calculate turn angle
        if not self.emergency:
            self.car.forward(40)

    def stop_car(self):
        self.car.forward(0)
        # TODO: This isn't working for some reason?
        # self.car.stop()

    def emergency_stop(self, stop_value):
        if stop_value:
            self.stop_car()
            self.emergency = True

    def set_angle(self, rel_dir):
        """Follow the line using grey scale camera"""
        self.car.set_dir_servo_angle(-1*rel_dir*self.scale)

    #FIXME: I never got this working properly
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
