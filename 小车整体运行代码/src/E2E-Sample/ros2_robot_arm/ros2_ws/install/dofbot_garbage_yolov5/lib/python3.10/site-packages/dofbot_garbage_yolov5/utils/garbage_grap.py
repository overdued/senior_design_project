#!/usr/bin/env python
# coding: utf-8

from time import sleep

import Arm_Lib
import serial
import time


data1 = "55 55 14 03 05 e8 03 01 b0 04 02 08 07 03 08 07 04 08 07 05 08 07"  # 张开
data2 = "55 55 14 03 05 e8 03 01 98 08 02 84 03 03 84 03 04 84 03 05 e8 03"  # 抓取
data3 = "55 55 14 03 05 e8 03 01 d0 07 02 dc 05 03 dc 05 04 dc 05 05 dc 05"  # 归位

data1 = data1.replace(" ","")
data1_hex = bytes.fromhex(data1)

data2 = data2.replace(" ","")
data2_hex = bytes.fromhex(data2)

data3 = data3.replace(" ","")
data3_hex = bytes.fromhex(data3)

ser = serial.Serial('/dev/ttyAMA1', 115200, timeout = 0.5)

def hand_grasp():
    ser.write(data2_hex)
    
def hand_open():
    ser.write(data1_hex)
    
def hand_default():
    ser.write(data3_hex)

class GarbageGrapMove:
    def __init__(self):
        # 设置移动状态
        self.move_status = True
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()
        # 夹爪加紧角度
        self.grap_joint = 130

    def move(self, joints, joints_down):
        """
        移动过程
        :param joints: 移动到物体位置的各关节角度
        :param joints_down: 机械臂抬起各关节角度
        """
        joints_uu = [joints[0], 80, 50, 50, 135, self.grap_joint]
        # 抬起
        joints_up = [joints_down[0], 80, 50, 50, 135, 30]
        joints_2 = [90, 80, 50, 50, 135,30]
        joints_3 = [180, 150,45, 0, 135,30]
        # 移动至物体位置上方
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        # 松开夹爪
        # self.arm.Arm_serial_servo_write(6, 0, 500)
        hand_open()
        sleep(0.5)
        # 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(joints, 500)
        sleep(0.5)
        # 进行抓取,夹紧夹爪
        # self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        hand_grasp()
        sleep(0.5)
        # 架起
        self.arm.Arm_serial_servo_write6_array(joints_2, 1000)
        sleep(1)
        # 抬起至对应位置上方
       # self.arm.Arm_serial_servo_write(1, joints_down[0], 500)
       # sleep(0.5)
        self.arm.Arm_serial_servo_write6_array(joints_3, 1000)
        sleep(1)
        # 释放物体,松开夹爪
        # self.arm.Arm_serial_servo_write(6, 30, 500)
        hand_open()
        sleep(1)
        # 抬起
        #self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        hand_default()
        sleep(1)
        
        

    def arm_run(self, name, joints,degree):
        """
        机械臂移动函数
        :param name:识别的垃圾名称
        :param joints: 反解求得的各关节角度
        """
        # 有害垃圾--红色
        if (
            name == "Plastic bag"
            or name == "Drink"
            or name == "Box"
            or name == "Computer mouse"
            and self.move_status
        ):
            # 此处设置,需执行完本次操作,才能向下运行
            self.move_status = False
            # 获得目标关节角
            joints = [joints[0]+degree-90, joints[1], joints[2], joints[3], 135, 30]
            # 移动到垃圾桶位置放下对应姿态
            joints_down = [45, 50, 20, 60, 135, self.grap_joint]
            # 移动
            self.move(joints, joints_down)
            # 移动完毕
            self.move_status = True
        # # 可回收垃圾--蓝色
        # if (
        #     name == "Zip_top_can"
        #     or name == "Newspaper"
        #     or name == "Old_school_bag"
        #     or name == "Book"
        #     and self.move_status
        # ):
        #     self.move_status = False
        #     joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
        #     joints_down = [27, 75, 0, 50, 265, self.grap_joint]
        #     self.move(joints, joints_down)
        #     self.move_status = True
        # # 厨余垃圾--绿色
        # if (
        #     name == "Fish_bone"
        #     or name == "Watermelon_rind"
        #     or name == "Apple_core"
        #     or name == "Egg_shell"
        #     and self.move_status
        # ):
        #     self.move_status = False
        #     joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
        #     joints_down = [147, 75, 0, 50, 265, self.grap_joint]
        #     self.move(joints, joints_down)
        #     self.move_status = True

        # # 其他垃圾--灰色
        # if (
        #     name == "Yellow"
        #     or name == "Cigarette_butts"
        #     or name == "Toilet_paper"
        #     or name == "Peach_pit"
        #     or name == "Disposable_chopsticks"
        #     and self.move_status
        # ):
        #     self.move_status = False
        #     joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
        #     joints_down = [133, 50, 20, 60, 265, self.grap_joint]
        #     self.move(joints, joints_down)
        #     self.move_status = True
