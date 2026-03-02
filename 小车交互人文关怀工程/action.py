from time import sleep
import Arm_Lib
import cv2
from pathlib import Path
import numpy as np
import serial   
from pydub import AudioSegment
from pydub.playback import play

arm = Arm_Lib.Arm_Device()
data1 = "55 55 14 03 05 e8 03 01 b0 04 02 08 07 03 08 07 04 08 07 05 08 07"  # 张开
data2 = "55 55 14 03 05 e8 03 01 98 08 02 84 03 03 84 03 04 84 03 05 e8 03"  # 抓取
data3 = "55 55 14 03 05 e8 03 01 dc 05 02 b0 04 03 b0 04 04 b0 04 05 b0 04"  # 握手

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
    
def hand_shake():
    ser.write(data3_hex)
    
def say_something(file_name):
    audio = AudioSegment.from_file(file_name, format = "m4a")
    adjusted_audio = audio.apply_gain(-10)
    play(adjusted_audio)

def dance1(): # 碰拳
    cap = cv2.VideoCapture(2)
    cap1=cv2.VideoCapture(0)
    D0 = [90,120, 60, 0,225, 30]
    D1=[60,120, 60, 0,225, 30]
    D2=[120,120, 60, 0,225, 30]
    D3=[90,120, 60, 0,225, 30]
    D4=[90,120, 60, 30,225, 30]
    D5=[90,120, 60, 0,225, 30]
    D6=[90,90, 60, 30,225, 30]
    D7=[90,150, 45, 0,225, 30]
    # 重置机械臂位置
    print("Start Reset Robot Arm Position, Please Wait..")
    sleep(5)
    hand_open()
    arm.Arm_serial_servo_write6_array(D0, 500)
    sleep(1)
    
    say_something("/root/机械臂代码/hei bro.m4a")
    
    arm.Arm_serial_servo_write6_array(D1, 500)
    sleep(1)
    arm.Arm_serial_servo_write6_array(D2, 500)
    sleep(1)
    arm.Arm_serial_servo_write6_array(D3, 500)
    sleep(1)
    hand_grasp()
    arm.Arm_serial_servo_write6_array(D4, 500)
    sleep(1)
    arm.Arm_serial_servo_write6_array(D5, 500)
    sleep(1)
    arm.Arm_serial_servo_write6_array(D6, 500)
    sleep(1)
    arm.Arm_serial_servo_write6_array(D7, 500)
    sleep(1)
    hand_open()
# sleep(2)
# arm.Arm_serial_servo_write6_array(joints_1, 1000)
def dance2():  # 握手
    sleep(5)
    
    hand_open()
    
    say_something("/root/机械臂代码/握手.m4a")
    
    D8=[90,150, 45, 10,225, 30]
    arm.Arm_serial_servo_write6_array(D8, 500)
    hand_shake()
    sleep(1)
    D9=[90,150, 45, 0,225, 30]
    arm.Arm_serial_servo_write6_array(D9, 500)
    sleep(1)
    D10=[90,150, 45, 20,225, 30]
    arm.Arm_serial_servo_write6_array(D10, 500)
    hand_open()
    
def dance3():  # 挥手
    D0 = [90, 120, 60, 0, 225, 30]  # 归位
    D1 = [0, 90, 90, 90, 225, 30]   # 手臂伸直
    D2 = [0, 90, 90, 60, 225, 30]   # 挥手1
    D3 = [0, 90, 90, 120, 225, 30]  # 挥手2
    
    sleep(5)
    arm.Arm_serial_servo_write6_array(D1, 500)
    sleep(1)
    say_something("/root/机械臂代码/你好.m4a")
    arm.Arm_serial_servo_write6_array(D2, 500)
    sleep(1)
    arm.Arm_serial_servo_write6_array(D3, 500)
    sleep(1)
    arm.Arm_serial_servo_write6_array(D1, 500)
    sleep(1)
    arm.Arm_serial_servo_write6_array(D0, 500)
    
    
if __name__ == "__main__":

    dance1()
    dance2()
    dance3()