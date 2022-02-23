#!/usr/bin/python3
# coding=utf8
import argparse
import sys
sys.path.append('ArmPi/')

import cv2
import time
import math
import threading
import logging
import numpy as np
from LABConfig import color_range
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
from camera import Camera

from arm.move import Move
from arm.shared_state import SharedState


class SharedState():
    '''Container class to share information between Move and Perception class'''
    def __init__(self, color) -> None:
        self.AK = ArmIK()
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }
        self.count = 0
        self.track = False
        self._stop = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__isRunning = False
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        self.move_square = False #TODO: Do we need this here?
        self.color_list = []

        self.__target_color = (color,)
        logging.debug("Set color: {}".format(self.__target_color))
        self.servo1 = 500
        self.rect = None
        self.size = (640, 480)
        self.rotation_angle = 0
        self.unreachable = False
        self.world_X = 0
        self.world_Y = 0
        self.world_x = 0
        self.world_y = 0
        self.t1 = 0
        self.roi = ()
        self.last_x = 0
        self.last_y = 0

        #Reset servos
        self.init()

    def init(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def start(self):
        #Reset state variables        
        self.count = 0
        self._stop = False
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        self.move_square = False #TODO: Do we need this here?
        self.color_list = []

        #Set is_Running flag
        self.__isRunning = True

    
    def stop(self):
        self._stop = True
        self.__isRunning = False