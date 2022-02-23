#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('ArmPi/')

import cv2
import time
import math
import logging
import numpy as np
from LABConfig import color_range
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from CameraCalibration.CalibrationConfig import *

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.ERROR , datefmt="%H:%M:%S ")


class Perception():
    def __init__(self, shared_state):
        self.state = shared_state
        

    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours: 
            contour_area_temp = math.fabs(cv2.contourArea(c)) 
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    area_max_contour = c

        return area_max_contour, contour_area_max

    #TODO: Below here are the perception methods
    def get_max_area(self, frame_lab, i):
        frame_mask = cv2.inRange(frame_lab, color_range[i][0], color_range[i][1])  # 对原图像和掩模进行位运算
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        areaMaxContour, area_max = self.getAreaMaxContour(contours)  # 找出最大轮廓
        return areaMaxContour, area_max
    
    def draw_box(self, box, img):
        cv2.drawContours(img, [box], -1, self.state.range_rgb[self.state.detect_color], 2)
        cv2.putText(img, '(' + str(self.state.world_x) + ',' + str(self.state.world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.state.range_rgb[self.state.detect_color], 1)
        distance = math.sqrt(pow(self.state.world_x - self.state.last_x, 2) + pow(self.state.world_y - self.state.last_y, 2))
        return img, distance

    def update_state(self):
        self.state.center_list.extend((self.state.world_x, self.state.world_y))
        self.state.count += 1
        if self.state.start_count_t1:
            self.state.start_count_t1 = False
            self.state.t1 = time.time()
        if time.time() - self.state.t1 > 1.5:
            self.state.rotation_angle = self.state.rect[2]
            self.state.start_count_t1 = True
            self.state.world_X, self.state.world_Y = np.mean(np.array(self.state.center_list).reshape(self.state.count, 2), axis=0)
            self.state.count = 0
            self.state.center_list = []
            self.state.start_pick_up = True

    def update_world_coord(self):
        img_centerx, img_centery = getCenter(self.state.rect, self.state.roi, self.state.size, square_length)
        self.state.world_x, self.state.world_y = convertCoordinate(img_centerx, img_centery, self.state.size)
        self.state.last_x, self.state.last_y = self.state.world_x, self.state.world_y

    def identify_multiple_colors(self, img):    
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        if not self.state.isRunning:
            return img

        frame_resize = cv2.resize(img_copy, self.state.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        if self.state.get_roi and not self.state.start_pick_up:
            self.state.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.state.roi, self.state.size)    
        
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
        max_area = 0
        areaMaxContour_max = 0
        
        if not self.state.start_pick_up:
            for i in color_range:
                if i in self.state.target_color:
                    areaMaxContour, area_max = self.get_max_area(frame_lab, i)
                    if areaMaxContour is not None:
                        if area_max > max_area:  # 找最大面积
                            max_area = area_max
                            self.state.detect_color = i
                            areaMaxContour_max = areaMaxContour
            if max_area > 2500:  # 有找到最大面积
                self.state.rect = cv2.minAreaRect(areaMaxContour_max)
                box = np.int0(cv2.boxPoints(self.state.rect))
                
                self.state.roi = getROI(box)
                self.state.get_roi = True
                self.update_world_coord()

                if not self.state.start_pick_up:
                    img, distance = self.draw_box(box, img)

                if not self.state.start_pick_up:
                    if self.state.detect_color == 'red':  # 红色最大
                        color = 1
                    elif self.state.detect_color == 'green':  # 绿色最大
                        color = 2
                    elif self.state.detect_color == 'blue':  # 蓝色最大
                        color = 3
                    else:
                        color = 0
                    self.state.color_list.append(color)

                    # 累计判断
                    if distance < 0.5:
                        self.update_state()
                    else:
                        self.state.t1 = time.time()
                        self.state.start_count_t1 = True
                        self.state.count = 0
                        self.state.center_list = []

                    if len(self.state.color_list) == 3:  # 多次判断
                        # 取平均值
                        color = int(round(np.mean(np.array(self.state.color_list))))
                        self.state.color_list = []
                        if color == 1:
                            self.state.detect_color = 'red'
                        elif color == 2:
                            self.state.detect_color = 'green'
                        elif color == 3:
                            self.state.detect_color = 'blue'
                        else:
                            self.state.detect_color = 'black'
            else:
                if not self.state.start_pick_up:
                    self.state.detect_color = "None"
        
        if self.state.move_square:
            cv2.putText(img, "Make sure no blocks in the stacking area", (15, int(img.shape[0]/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)    
        
        if self.state.detect_color != "None":
            cv2.putText(img, "Color: " + self.state.detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.state.range_rgb[self.state.detect_color], 2)
        
        return img
        
    def identify_single_color(self, img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        
        if not self.state.isRunning:
            return img
        
        frame_resize = cv2.resize(img_copy, self.state.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        if self.state.get_roi and self.state.start_pick_up:
            self.state.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.state.roi, self.state.size)    
        
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB) 
        
        area_max = 0
        areaMaxContour = 0
        if not self.state.start_pick_up:
            for i in color_range:
                if i in self.state.target_color:
                    self.state.detect_color = i
                    areaMaxContour, area_max = self.get_max_area(frame_lab, i)
                    
                   
            if area_max > 2500:  # 有找到最大面积
                self.state.rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(self.state.rect))

                self.state.roi = getROI(box)
                self.state.get_roi = True

                self.update_world_coord()
                self.state.track = True

                img, distance = self.draw_box(box, img)

                if self.state.action_finish:
                    if distance < 0.3:
                        self.update_state()
                    else:
                        self.state.t1 = time.time()
                        self.state.start_count_t1 = True
                        self.state.count = 0
                        self.state.center_list = []
        return img
    
