#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('ArmPi/')

import logging
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *


class SharedState():
    '''Container class to share information between Move and Perception class'''
    def __init__(self, target_colors, multi_color_flag) -> None:
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
        self.isRunning = False
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        self.move_square = False #TODO: Do we need this here?
        self.color_list = []

        self.target_color = target_colors
        if int(multi_color_flag) <= 1:
            self.target_color = [self.target_color[0]]
        logging.debug("Set color: {}".format(self.target_color))

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
        self.isRunning = True

    
    def stop(self):
        self._stop = True
        self.isRunning = False