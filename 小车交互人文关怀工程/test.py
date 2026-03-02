from time import sleep
import Arm_Lib
import cv2 as cv
from pathlib import Path
import numpy as np
import serial   
xy = [270, 135]
arm = Arm_Lib.Arm_Device()
joints_0 = [0,120, 0, 0, 135, 30]

joints_2 = [90, 80, 50, 50, 135,30]
joints_1 = [180, 150,45, 0, 135,30]
# 重置机械臂位置
print("Start Reset Robot Arm Position, Please Wait..")
arm.Arm_serial_servo_write6_array(joints_2, 1000)
sleep(2)
arm.Arm_serial_servo_write6_array(joints_1, 1000)
#arm.Arm_serial_servo_write6_array(joints_0, 1000)
sleep(2)
