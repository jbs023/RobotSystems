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

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.ERROR , datefmt="%H:%M:%S ")


class Arm():
    def __init__(self, color):
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

        #Set is_Running flag
        self.__isRunning = True

    
    def stop(self):
        self._stop = True
        self.__isRunning = False

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

    def setBuzzer(self,timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)

    def set_rgb(self, color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()

    # Primary method 1
    def move(self):
        # 不同颜色木快放置坐标(x, y, z)
        logging.debug("start move thread")
        coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }
        while True:
            if self.__isRunning:
                if self.first_move and self.start_pick_up:               
                    self.action_finish = False
                    self.set_rgb(self.detect_color)
                    self.setBuzzer(0.1)               
                    result = self.AK.setPitchRangeMoving((self.world_X, self.world_Y - 2, 5), -90, -90, 0) # 不填运行时间参数，自适应运行时间
                    if result == False:
                        self.unreachable = True
                    else:
                        self.unreachable = False
                    time.sleep(result[2]/1000) # 返回参数的第三项为时间
                    self.start_pick_up = False
                    self.first_move = False
                    self.action_finish = True
                elif not self.first_move and not self.unreachable: # 不是第一次检测到物体
                    self.set_rgb(self.detect_color)
                    if self.track: # 如果是跟踪阶段
                        if not self.__isRunning: # 停止以及退出标志位检测
                            continue
                        self.AK.setPitchRangeMoving((self.world_x, self.world_y - 2, 5), -90, -90, 0, 20)
                        time.sleep(0.02)                    
                        self.track = False
                    if self.start_pick_up: #如果物体没有移动一段时间，开始夹取
                        self.action_finish = False
                        if not self.__isRunning: # 停止以及退出标志位检测
                            continue
                        Board.setBusServoPulse(1, self.servo1 - 280, 500)  # 爪子张开
                        # 计算夹持器需要旋转的角度
                        servo2_angle = getAngle(self.world_X, self.world_Y, self.rotation_angle)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.8)
                        
                        if not self.__isRunning:
                            continue
                        self.AK.setPitchRangeMoving((self.world_X, self.world_Y, 2), -90, -90, 0, 1000)  # 降低高度
                        time.sleep(2)
                        
                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.servo1, 500)  # 夹持器闭合
                        time.sleep(1)
                        
                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        self.AK.setPitchRangeMoving((self.world_X, self.world_Y, 12), -90, -90, 0, 1000)  # 机械臂抬起
                        time.sleep(1)
                        
                        if not self.__isRunning:
                            continue
                        # 对不同颜色方块进行分类放置
                        result = self.AK.setPitchRangeMoving((coordinate[self.detect_color][0], coordinate[self.detect_color][1], 12), -90, -90, 0)   
                        time.sleep(result[2]/1000)
                        
                        if not self.__isRunning:
                            continue
                        servo2_angle = getAngle(coordinate[self.detect_color][0], coordinate[self.detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not self.__isRunning:
                            continue
                        self.AK.setPitchRangeMoving((coordinate[self.detect_color][0], coordinate[self.detect_color][1], coordinate[self.detect_color][2] + 3), -90, -90, 0, 500)
                        time.sleep(0.5)
                        
                        if not self.__isRunning:
                            continue
                        self.AK.setPitchRangeMoving((coordinate[self.detect_color]), -90, -90, 0, 1000)
                        time.sleep(0.8)
                        
                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.servo1 - 200, 500)
                        time.sleep(0.8)
                        
                        if not self.__isRunning:
                            continue                    
                        self.AK.setPitchRangeMoving((coordinate[self.detect_color][0], coordinate[self.detect_color][1], 12), -90, -90, 0, 800)
                        time.sleep(0.8)

                        self.init()  #reset servos
                        time.sleep(1.5)

                        self.detect_color = 'None'
                        self.first_move = True
                        self.get_roi = False
                        self.action_finish = True
                        self.start_pick_up = False
                        self.set_rgb(self.detect_color)
                    else:
                        time.sleep(0.01)
            else:
                if self._stop:
                    self._stop = False
                    Board.setBusServoPulse(1, self.servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)

    def run(self, img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        
        if not self.__isRunning:
            return img
        
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        #如果检测到某个区域有识别到的物体，则一直检测该区域直到没有为止
        if self.get_roi and self.start_pick_up:
            self.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)    
        
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
        
        area_max = 0
        areaMaxContour = 0
        if not self.start_pick_up:
            for i in color_range:
                logging.debug("color: {}, target_color: {}".format(i, self.__target_color))
                if i in self.__target_color:
                    logging.debug("Looking for color: {}".format(i))
                    self.detect_color = i
                    frame_mask = cv2.inRange(frame_lab, color_range[self.detect_color][0], color_range[self.detect_color][1])  # 对原图像和掩模进行位运算
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
                    areaMaxContour, area_max = self.getAreaMaxContour(contours)  # 找出最大轮廓
                    logging.debug("Largest contour: {}".format(area_max))
            if area_max > 2500:  # 有找到最大面积
                logging.debug("Color found")
                self.rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(self.rect))

                self.roi = getROI(box)
                self.get_roi = True

                img_centerx, img_centery = getCenter(self.rect, self.roi, self.size, square_length)  # 获取木块中心坐标
                self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, self.size) #转换为现实世界坐标
                
                cv2.drawContours(img, [box], -1, self.range_rgb[self.detect_color], 2)
                cv2.putText(img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[self.detect_color], 1) #绘制中心点
                distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2)) #对比上次坐标来判断是否移动
                self.last_x, self.last_y = self.world_x, self.world_y
                self.track = True

                if self.action_finish:
                    if distance < 0.3:
                        self.center_list.extend((self.world_x, self.world_y))
                        self.count += 1
                        if self.start_count_t1:
                            self.start_count_t1 = False
                            self.t1 = time.time()
                        if time.time() - self.t1 > 1.5:
                            self.rotation_angle = self.rect[2]
                            self.start_count_t1 = True
                            self.world_X, self.world_Y = np.mean(np.array(self.center_list).reshape(self.count, 2), axis=0)
                            self.count = 0
                            self.center_list = []
                            self.start_pick_up = True
                    else:
                        self.t1 = time.time()
                        self.start_count_t1 = True
                        self.count = 0
                        self.center_list = []
        return img

def main(config):
    #Line following simultaneity
    if config.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    #Init arm and camera objects
    arm = Arm('red')
    camera = Camera()
    arm.start()
    camera.camera_open()

    # Use the threads the same way original code did
    # they share too much information to quickly integrate
    # a consumer-producer framework

    #Start move thread
    move_thread = threading.Thread(target=arm.move, daemon=True)
    move_thread.start()

    #Start camera thread
    camera_threa = threading.Thread(target=camera.camera_task, args=(), daemon=True)
    camera_threa.start()

    #Start main thread, which executes the run function
    while True:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = arm.run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    camera.camera_close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Debug flag')
    main(parser.parse_args())
    
