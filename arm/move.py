#!/usr/bin/python3
# coding=utf8
import time
import sys
sys.path.append('ArmPi/')


from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *


class Move():
    def __init__(self, shared_state):
        self.state = shared_state
        self.coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }

    def setBuzzer(self,timer):
        Board.self.setBuzzer(0)
        Board.self.setBuzzer(1)
        time.sleep(timer)
        Board.self.setBuzzer(0)

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

    def move_block(self):
        '''Move one block'''
        while True:
            if self.state.__isRunning:
                if self.state.first_move and self.state.start_pick_up:               
                    self.state.action_finish = False
                    self.set_rgb(self.state.detect_color)
                    self.setBuzzer(0.1)               
                    result = self.state.AK.setPitchRangeMoving((self.state.world_X, self.state.world_Y - 2, 5), -90, -90, 0) # 不填运行时间参数，自适应运行时间
                    if result == False:
                        self.state.unreachable = True
                    else:
                        self.state.unreachable = False
                    time.sleep(result[2]/1000) # 返回参数的第三项为时间
                    self.state.start_pick_up = False
                    self.state.first_move = False
                    self.state.action_finish = True
                elif not self.state.first_move and not self.state.unreachable: # 不是第一次检测到物体
                    self.set_rgb(self.state.detect_color)
                    if self.state.track: # 如果是跟踪阶段
                        if not self.state.__isRunning: # 停止以及退出标志位检测
                            continue
                        self.state.AK.setPitchRangeMoving((self.state.world_x, self.state.world_y - 2, 5), -90, -90, 0, 20)
                        time.sleep(0.02)                    
                        self.state.track = False
                    if self.state.start_pick_up: #如果物体没有移动一段时间，开始夹取
                        self.state.action_finish = False
                        if not self.state.__isRunning: # 停止以及退出标志位检测
                            continue
                        Board.setBusServoPulse(1, self.state.servo1 - 280, 500)  # 爪子张开
                        # 计算夹持器需要旋转的角度
                        servo2_angle = getAngle(self.state.world_X, self.state.world_Y, self.state.rotation_angle)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.8)
                        
                        if not self.state.__isRunning:
                            continue
                        self.state.AK.setPitchRangeMoving((self.state.world_X, self.state.world_Y, 2), -90, -90, 0, 1000)  # 降低高度
                        time.sleep(2)
                        
                        if not self.state.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.state.servo1, 500)  # 夹持器闭合
                        time.sleep(1)
                        
                        if not self.state.__isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        self.state.AK.setPitchRangeMoving((self.state.world_X, self.state.world_Y, 12), -90, -90, 0, 1000)  # 机械臂抬起
                        time.sleep(1)
                        
                        if not self.state.__isRunning:
                            continue
                        # 对不同颜色方块进行分类放置
                        result = self.state.AK.setPitchRangeMoving((self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], 12), -90, -90, 0)   
                        time.sleep(result[2]/1000)
                        
                        if not self.state.__isRunning:
                            continue
                        servo2_angle = getAngle(self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not self.state.__isRunning:
                            continue
                        self.state.AK.setPitchRangeMoving((self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], self.coordinate[self.state.detect_color][2] + 3), -90, -90, 0, 500)
                        time.sleep(0.5)
                        
                        if not self.state.__isRunning:
                            continue
                        self.state.AK.setPitchRangeMoving((self.coordinate[self.state.detect_color]), -90, -90, 0, 1000)
                        time.sleep(0.8)
                        
                        if not self.state.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.state.servo1 - 200, 500)
                        time.sleep(0.8)
                        
                        if not self.state.__isRunning:
                            continue                    
                        self.state.AK.setPitchRangeMoving((self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], 12), -90, -90, 0, 800)
                        time.sleep(0.8)

                        self.state.init()  #reset servos
                        time.sleep(1.5)

                        self.state.detect_color = 'None'
                        self.state.first_move = True
                        self.state.get_roi = False
                        self.state.action_finish = True
                        self.state.start_pick_up = False
                        self.set_rgb(self.state.detect_color)
                    else:
                        time.sleep(0.01)
            else:
                if self.state._stop:
                    self.state._stop = False
                    Board.setBusServoPulse(1, self.state.servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    self.state.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)


    def sort_blocks(self):
        '''Move blocks one at a time'''
        while True:
            if self.state.__isRunning:        
                if self.state.detect_color != 'None' and self.state.start_pick_up:
                    self.set_rgb(self.state.detect_color)
                    self.setBuzzer(0.1)
                    result = self.state.AK.setPitchRangeMoving((self.state.world_X, self.state.world_Y, 7), -90, -90, 0)  
                    if result == False:
                        self.state.unreachable = True
                    else:
                        self.state.unreachable = False
                        time.sleep(result[2]/1000)

                        if not self.state.__isRunning:
                            continue
                        servo2_angle = getAngle(self.state.world_X, self.state.world_Y, self.state.rotation_angle) #计算夹持器需要旋转的角度
                        Board.setBusServoPulse(1, self.state.servo1 - 280, 500)  # 爪子张开
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)
                        
                        if not self.state.__isRunning:
                            continue
                        self.state.AK.setPitchRangeMoving((self.state.world_X, self.state.world_Y, 1.5), -90, -90, 0, 1000)
                        time.sleep(1.5)

                        if not self.state.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.state.servo1, 500)  #夹持器闭合
                        time.sleep(0.8)

                        if not self.state.__isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        self.state.AK.setPitchRangeMoving((self.state.world_X, self.state.world_Y, 12), -90, -90, 0, 1000)  #机械臂抬起
                        time.sleep(1)

                        if not self.state.__isRunning:
                            continue
                        result = self.state.AK.setPitchRangeMoving((self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], 12), -90, -90, 0)   
                        time.sleep(result[2]/1000)
                        
                        if not self.state.__isRunning:
                            continue                   
                        servo2_angle = getAngle(self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not self.state.__isRunning:
                            continue
                        self.state.AK.setPitchRangeMoving((self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], self.coordinate[self.state.detect_color][2] + 3), -90, -90, 0, 500)
                        time.sleep(0.5)
                        
                        if not self.state.__isRunning:
                            continue                    
                        self.state.AK.setPitchRangeMoving((self.coordinate[self.state.detect_color]), -90, -90, 0, 1000)
                        time.sleep(0.8)

                        if not self.state.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.state.servo1 - 200, 500)  # 爪子张开  ，放下物体
                        time.sleep(0.8)

                        if not self.state.__isRunning:
                            continue
                        self.state.AK.setPitchRangeMoving((self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], 12), -90, -90, 0, 800)
                        time.sleep(0.8)

                        self.state.init()  # 回到初始位置
                        time.sleep(1.5)

                        self.state.detect_color = 'None'
                        self.state.get_roi = False
                        self.state.start_pick_up = False
                        self.set_rgb(self.state.detect_color)
            else:
                if self.state._stop:
                    self.state._stop = False
                    Board.setBusServoPulse(1, self.state.servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    self.state.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)
            
    def palletize_blocks(self):
        '''Stack blocks'''
        dz = 2.5
        z = self.coordinate['red'][2] if self.state.detect_color == 'None' else self.coordinate['red'][self.state.detect_color]

        while True:
            if self.state.__isRunning:
                if self.state.detect_color != 'None' and self.state.start_pick_up:  # 如果检测到方块没有移动一段时间后，开始夹取
                    self.set_rgb(self.state.detect_color)
                    self.setBuzzer(0.1)
                    # 高度累加
                    z = z_r
                    z_r += dz
                    if z == 2 * dz + self.coordinate['red'][2]:
                        z_r = self.coordinate['red'][2]
                    if z == self.coordinate['red'][2]:  
                        move_square = True
                        time.sleep(3)
                        move_square = False
                    result = self.state.AK.setPitchRangeMoving((self.state.world_X, self.state.world_Y, 7), -90, -90, 0)  # 移到目标位置，高度5cm
                    if result == False:
                        self.state.unreachable = True
                    else:
                        self.state.unreachable = False
                        time.sleep(result[2]/1000)

                        if not self.state.__isRunning:
                            continue
                        # 计算夹持器需要旋转的角度
                        servo2_angle = getAngle(self.state.world_X, self.state.world_Y, self.state.rotation_angle)
                        Board.setBusServoPulse(1, self.state.servo1 - 280, 500)  # 爪子张开
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not self.state.__isRunning:
                            continue
                        self.state.AK.setPitchRangeMoving((self.state.world_X, self.state.world_Y, 2), -90, -90, 0, 1000)  # 降低高度到2cm
                        time.sleep(1.5)

                        if not self.state.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.state.servo1, 500)  # 夹持器闭合
                        time.sleep(0.8)

                        if not self.state.__isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        self.state.AK.setPitchRangeMoving((self.state.world_X, self.state.world_Y, 12), -90, -90, 0, 1000)  # 机械臂抬起
                        time.sleep(1)

                        if not self.state.__isRunning:
                            continue
                        self.state.AK.setPitchRangeMoving((self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], 12), -90, -90, 0, 1500) 
                        time.sleep(1.5)
                        
                        if not self.state.__isRunning:
                            continue                  
                        servo2_angle = getAngle(self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not self.state.__isRunning:
                            continue
                        self.state.AK.setPitchRangeMoving((self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], z + 3), -90, -90, 0, 500)
                        time.sleep(0.5)
                        
                        if not self.state.__isRunning:
                            continue                
                        self.state.AK.setPitchRangeMoving((self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], z), -90, -90, 0, 1000)
                        time.sleep(0.8)

                        if not self.state.__isRunning:
                            continue 
                        Board.setBusServoPulse(1, self.state.servo1 - 200, 500)  # 爪子张开  ，放下物体
                        time.sleep(1)

                        if not self.state.__isRunning:
                            continue 
                        self.state.AK.setPitchRangeMoving((self.coordinate[self.state.detect_color][0], self.coordinate[self.state.detect_color][1], 12), -90, -90, 0, 800)
                        time.sleep(0.8)

                        self.state.init()  # 回到初始位置
                        time.sleep(1.5)

                        self.state.detect_color = 'None'
                        self.state.get_roi = False
                        self.state.start_pick_up = False
                        self.set_rgb(self.state.detect_color)
            else:
                if self.state._stop:
                    self.state._stop = False
                    Board.setBusServoPulse(1, self.state.servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    self.state.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)               
                time.sleep(0.01)