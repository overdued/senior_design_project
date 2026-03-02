
#!/usr/bin/env python
# coding: utf-8

import os
from time import sleep

import Arm_Lib
import cv2 as cv
from pathlib import Path
import numpy as np
import serial
import time
from .utils.dofbot_config import ArmCalibration, read_XYT
from .utils.garbage_identify import GarbageIdentify

import os
import time
import csv
import cv2
import numpy as np
import math
import torch
import serial
from openni import openni2
from .test import YOLO_OI601

data1 = "55 55 14 03 05 e8 03 01 b0 04 02 08 07 03 08 07 04 08 07 05 08 07"  # 张开
data2 = "55 55 14 03 05 e8 03 01 98 08 02 84 03 03 84 03 04 84 03 05 e8 03"  # 抓取

data1 = data1.replace(" ","")
data1_hex = bytes.fromhex(data1)

data2 = data2.replace(" ","")
data2_hex = bytes.fromhex(data2)

ser = serial.Serial('/dev/ttyAMA1', 115200, timeout = 0.5)

def hand_grasp():
    ser.write(data2_hex)
    
def hand_open():
    ser.write(data1_hex)

def is_box_in_center_region(x1, y1, x2, y2, img_width=640, img_height=480, center_ratio=0.4):
    """
    检查检测框是否在图像中心区域附近
    :param x1, y1, x2, y2: 检测框坐标 (像素)
    :param img_width, img_height: 图像尺寸
    :param center_ratio: 中心区域占整个图像的比例
    :return: True if box center is in center region
    """
    # 计算检测框中心点
    box_center_x = (x1 + x2) / 2
    box_center_y = (y1 + y2) / 2
    
    # 计算图像中心点
    img_center_x = img_width / 2
    img_center_y = img_height / 2
    
    # 计算中心区域的范围
    center_width = img_width * center_ratio
    center_height = img_height * center_ratio
    
    # 计算中心区域的边界
    center_left = img_center_x - center_width / 2
    center_right = img_center_x + center_width / 2
    center_top = img_center_y - center_height / 2
    center_bottom = img_center_y + center_height / 2
    
    # 检查检测框中心是否在中心区域内
    is_in_center = (center_left <= box_center_x <= center_right and 
                   center_top <= box_center_y <= center_bottom)
    
    print(f"检测框中心: ({box_center_x:.1f}, {box_center_y:.1f})")
    print(f"图像中心区域: ({center_left:.1f}, {center_top:.1f}) - ({center_right:.1f}, {center_bottom:.1f})")
    print(f"Box是否在中心区域: {is_in_center}")
    
    return is_in_center

def Grap(args=None):
    # 创建获取目标实例
    target = GarbageIdentify()
    # 创建相机标定实例
    calibration = ArmCalibration()
    # 创建机械臂驱动实例
    arm = Arm_Lib.Arm_Device()
    # 初始化一些参数
    # 初始化标定方框边点
    dp = []
    # 初始化抓取信息
    msg = {}
    # 初始化1,2舵机角度值
    xy = [90, 135]
    # 是否打印透视变换参数
    DP_PRINT = False
    # 预热值
    WARMUP_BUFFER = 10
    Grapfinishstate = False
    GrapFindstate = 1
    Finddegree = 0
    FILE = Path(__file__).resolve()
    lib_root = FILE.parents[0]
    lib_site_pkg = os.path.dirname(lib_root)
    lib_python = os.path.dirname(lib_site_pkg)
    lib_path = os.path.dirname(lib_python)
    shared_path = os.path.join(os.path.dirname(lib_path), "share")
    share_root = os.path.join(shared_path, "dofbot_garbage_yolov5")
    cfg_folder = os.path.join(share_root, "config")
    dp_cfg_path = os.path.join(cfg_folder, "dp.bin")
    
    # 新增的控制变量
    inference_count = 0  # 当前角度的推理次数
    max_inference_per_angle = 10  # 每个角度最大推理次数
    current_angle = 90  # 初始角度设为90度
    angle_step = 10  # 角度步进
    min_angle = 30   # 最小角度
    max_angle = 120  # 最大角度
    initial_search_done = False  # 标记是否已完成90度的初始搜索
    
    # 连续识别控制变量
    consecutive_box_count = 0  # 连续识别到Box的次数
    required_consecutive_count = 5  # 需要连续识别的次数
    last_box_angle = 90  # 上次识别到Box的角度
    
    # 中心区域检测参数
    center_check_enabled = True  # 是否启用中心区域检测
    center_ratio = 0.4  # 中心区域占图像的比例（40%）
    
    # XYT参数路径
    XYT_path = os.path.join(cfg_folder, "XYT_config.txt")
    try:
        xy, _ = read_XYT(XYT_path)
    except Exception:
        print("Read XYT_config Error !!!")
        return

    print("Read xy is", xy)

    warm_up_count = 0
    last_num = 0
    last_count = 0

    arm = Arm_Lib.Arm_Device()
    joints_0 = [xy[0], xy[1], 0, 0, 135, 30]
    hand_open()
    joints_1 = [xy[0], xy[1], 50, 50, 135, 30]

    # 重置机械臂位置
    print("Start Reset Robot Arm Position, Please Wait..")
    arm.Arm_serial_servo_write6_array(joints_1, 1000)
    sleep(2)
    arm.Arm_serial_servo_write6_array(joints_0, 1000)
    sleep(2)
    print("Finish Robot Arm Position Reset!")
    
    # 打开摄像头
    capture = cv.VideoCapture(2)
    
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened() and not Grapfinishstate:
        # 读取相机的每一帧
        Findstate = False
        ret, img = capture.read()
        print("read image from camera successfully:", ret)
        if not ret:
            print("Failed to read image from camera")
            continue
            
        # 统一图像大小
        img = cv.resize(img, (640, 480))

        # 处理透视变换参数
        try:
            # 检查dp.bin文件是否存在
            if os.path.exists(dp_cfg_path):
                dp = np.fromfile(dp_cfg_path, dtype=np.int32)
                
                # 修复reshape问题：检查dp的大小并正确处理
                if len(dp) >= 8:
                    # 取前8个元素reshape为4x2
                    dp_temp = dp[:8].reshape(4, 2)
                    # 转换为正确的格式，确保是float32类型
                    dp_points = []
                    for i in range(4):
                        dp_points.append([float(dp_temp[i][0]), float(dp_temp[i][1])])
                    dp = np.array(dp_points, dtype=np.float32)
                else:
                    # 如果数据不足，使用默认值
                    dp = np.array([[100.0, 100.0], [540.0, 100.0], [540.0, 380.0], [100.0, 380.0]], dtype=np.float32)
            else:
                # 如果文件不存在，使用默认值
                dp = np.array([[100.0, 100.0], [540.0, 100.0], [540.0, 380.0], [100.0, 380.0]], dtype=np.float32)
                
            # 验证dp数组的形状和类型
            if dp.shape != (4, 2):
                if dp.size >= 8:
                    dp = dp.flatten()[:8].reshape(4, 2).astype(np.float32)
                else:
                    dp = np.array([[100.0, 100.0], [540.0, 100.0], [540.0, 380.0], [100.0, 380.0]], dtype=np.float32)
            
            # 确保数据类型正确
            dp = dp.astype(np.float32)
            
            # 执行透视变换
            img = calibration.perspective_transform(dp, img)

        except Exception as e:
            print(f"Perspective transform error: {e}")
            print("Using original image without perspective transform")

        # 执行垃圾识别
        try:
            # 移动机械臂到当前角度
            print(f"移动机械臂到角度: {current_angle}")
            arm.Arm_serial_servo_write(1, current_angle, 500)

            
            # 执行推理
            img, msg = target.garbage_run(img)
            inference_count += 1  # 增加推理计数
            
            print(f"角度 {current_angle}, 第 {inference_count} 次推理")
            
            if len(msg) != 0:
                # 检查msg中是否包含"Box"物体
                object_names = list(msg.keys())
                box_found = False
                box_in_center = False  # 标记Box是否在中心区域
                box_coords = None  # 保存Box的坐标
                
                for name in object_names:
                    if "Box" in name or name == "Box":  # 检查是否包含Box
                        box_found = True
                        Finddegree = current_angle
                        print("find degree:", Finddegree)
                        
                        # 检查Box是否在中心区域
                        if center_check_enabled and hasattr(target, 'last_detection_coords'):
                            # 获取Box的检测框坐标
                            if name in target.last_detection_coords:
                                x1, y1, x2, y2 = target.last_detection_coords[name]
                                box_coords = (x1, y1, x2, y2)
                                box_in_center = is_box_in_center_region(x1, y1, x2, y2, 640, 480, center_ratio)
                                print(f"Box坐标: ({x1}, {y1}, {x2}, {y2})")
                        else:
                            # 如果没有坐标信息或不启用中心检测，默认认为在中心
                            box_in_center = True
                        
                        break
                
                if box_found and box_in_center:
                    # 连续识别到Box且在中心区域，增加计数
                    # 检查是否是同一角度的连续识别
                    if Finddegree == last_box_angle:
                        consecutive_box_count += 1
                        print(f"连续第 {consecutive_box_count} 次识别到中心区域的Box在角度 {Finddegree}")
                    else:
                        # 角度发生变化，重置计数
                        consecutive_box_count = 1
                        last_box_angle = Finddegree
                        print(f"在新角度 {Finddegree} 第1次识别到中心区域的Box")
                    
                    # 检查是否达到连续识别次数要求
                    if consecutive_box_count >= required_consecutive_count:
                        print(f"连续 {required_consecutive_count} 次识别到中心区域的Box，执行抓取！")
                        target.garbage_grap(msg, xy, Finddegree)
                        Grapfinishstate = True
                        break  # 找到目标，退出循环
                    else:
                        print(f"还需 {required_consecutive_count - consecutive_box_count} 次连续识别才执行抓取")
                        # 继续当前角度的搜索
                        continue
                elif box_found and not box_in_center:
                    # 识别到Box但不在中心区域
                    consecutive_box_count = 0
                    print("识别到Box但不在中心区域，重置连续识别计数")
                    print("继续搜索，等待Box移动到中心区域")
                    # 继续当前角度的搜索
                    continue
                else:
                    # 识别到其他物体但不是Box，重置连续计数
                    consecutive_box_count = 0
                    print("识别到其他物体，重置连续识别计数")
                    
                    if not initial_search_done:
                        # 还在90度初始搜索阶段
                        if inference_count < max_inference_per_angle:
                            print(f"在90度位置识别到其他物体，继续第 {inference_count + 1} 次推理")
                            continue
                        else:
                            print(f"在90度位置完成 {max_inference_per_angle} 次推理，未找到Box，开始遍历搜索")
                            initial_search_done = True
                            inference_count = 0
                            current_angle = min_angle  # 开始从30度遍历
                    else:
                        # 遍历搜索阶段
                        if inference_count < max_inference_per_angle:
                            print(f"在角度 {current_angle} 识别到其他物体，继续第 {inference_count + 1} 次推理")
                            continue
                        else:
                            print(f"在角度 {current_angle} 完成 {max_inference_per_angle} 次推理，未找到Box，转向下一个角度")
                            # 转向下一个角度
                            inference_count = 0
                            consecutive_box_count = 0  # 重置连续计数
                            current_angle += angle_step
                            if current_angle > max_angle:
                                current_angle = min_angle  # 重置到起始角度
                                print("已完成一轮遍历搜索，重新开始")
            else:
                # msg为空，没有识别到任何物体
                # 重置连续识别计数
                consecutive_box_count = 0
                print("未识别到任何物体，重置连续识别计数")
                
                if not initial_search_done:
                    # 还在90度初始搜索阶段
                    if inference_count < max_inference_per_angle:
                        print(f"在90度位置未识别到物体，继续第 {inference_count + 1} 次推理")
                        continue
                    else:
                        print(f"在90度位置完成 {max_inference_per_angle} 次推理，未找到目标，开始遍历搜索")
                        initial_search_done = True
                        inference_count = 0
                        current_angle = min_angle  # 开始从30度遍历
                else:
                    # 遍历搜索阶段
                    if inference_count < max_inference_per_angle:
                        print(f"在角度 {current_angle} 未识别到物体，继续第 {inference_count + 1} 次推理")
                        continue
                    else:
                        print(f"在角度 {current_angle} 完成 {max_inference_per_angle} 次推理，未找到目标，转向下一个角度")
                        # 转向下一个角度
                        inference_count = 0
                        current_angle += angle_step
                        if current_angle > max_angle:
                            current_angle = min_angle  # 重置到起始角度
                            print("已完成一轮遍历搜索，重新开始")
                        
        except Exception as e:
            print(f"Error occurred: {e}")
            import traceback
            traceback.print_exc()
            # 出现异常时重置连续识别计数
            consecutive_box_count = 0
            
            # 出现异常时的处理
            if not initial_search_done:
                # 还在90度初始搜索阶段
                if inference_count >= max_inference_per_angle:
                    print("90度位置出现异常且已达到最大推理次数，开始遍历搜索")
                    initial_search_done = True
                    inference_count = 0
                    current_angle = min_angle
            else:
                # 遍历搜索阶段
                if inference_count >= max_inference_per_angle:
                    print(f"角度 {current_angle} 出现异常且已达到最大推理次数，转向下一个角度")
                    inference_count = 0
                    current_angle += angle_step
                    if current_angle > max_angle:
                        current_angle = min_angle
                        print("已完成一轮遍历搜索，重新开始")
      
    return


# ========== 配置 ==========
MODEL = "/root/ascend-devkit/src/E2E-Sample/ros2_robot_arm/ros2_ws/src/dofbot_garbage_yolov5/dofbot_garbage_yolov5/yolov8l.om"
CLASS_PATH = "/root/ascend-devkit/src/E2E-Sample/ros2_robot_arm/ros2_ws/src/dofbot_garbage_yolov5/dofbot_garbage_yolov5/classes.txt"
YOLO_INPUT_SIZE = 640

SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

# 类别ID
TARGET_CLASS_IDS_OBJECT = {345,62}   # 要抓的物体
TARGET_CLASS_IDS_BEGIN  = {381}   # 原点标志物

# 相机参数
IMG_WIDTH, IMG_HEIGHT = 640, 480
CAMERA_HEIGHT = 0.085
FOCAL_LENGTH = 740
CAMERA_PITCH_DEG = 15
CAMERA_PITCH_RAD = np.radians(CAMERA_PITCH_DEG)

# ================= 串口封装 =================
class SerialHandler:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.05)

    def send(self, msg):
        self.ser.write((msg + "\n").encode("utf-8"))
        print(f"[→UART] {msg}")

    def recv(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode("utf-8").strip()
            if line:
                print(f"[←UART] {line}")
            return line
        return None

    def close(self):
        self.ser.close()

# ================= 工具函数 =================
def pixel_to_horizontal_distance(cx, cy, depth_value, depth_stream):
    if depth_value <= 0:
        return None
    Xmm, Ymm, Zmm = openni2.convert_depth_to_world(depth_stream, cx, cy, int(depth_value))
    Z_d = Zmm / 1000.0
    Z_horizontal = np.sqrt(max(Z_d**2 - CAMERA_HEIGHT**2, 0.0))
    return round(Z_horizontal, 3)

# ================= 主程序 =================
def main():
    serh = SerialHandler(SERIAL_PORT, BAUD_RATE)

    # OpenNI 初始化
    openni2.initialize("/root/ascend-devkit/src/E2E-Sample/ros2_robot_arm/ros2_ws/src/dofbot_garbage_yolov5/dofbot_garbage_yolov5/yolov5_d435i_detection-main/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d/sdk/libs")
    dev = openni2.Device.open_any()
    depth_stream = dev.create_depth_stream()
    depth_stream.start()

    # 摄像头初始化
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 模型初始化
    print("加载 YOLO 模型中...")
    model = YOLO_OI601(MODEL, CLASS_PATH, conf_thr=0.25, iou_thr=0.5)
    print("✅ 模型加载成功")

    # 只有模型加载完成后才发 MOVE_BEGIN
    serh.send("MOVE_BEGIN")

    stage = "FIND_OBJECT"

    try:
        while True:
            # 先检查串口消息
            msg = serh.recv()
            if msg:
                if stage == "WAIT_ARRIVE" and msg == "arrive":
                    print("[FSM] 收到 arrive，等待5秒抓取")
                    Grap()
                    Grabfinishstate=False
                    serh.send("success")
                    stage = "FIND_BEGIN"

                elif stage == "WAIT_FINISH" and msg == "final_all":
                    print("[FSM] 收到 final_all，结束任务")
                    break

            # 摄像头帧
            ret, frame = cap.read()
            if not ret:
                break
            depth_frame = depth_stream.read_frame()
            depth_data = np.frombuffer(depth_frame.get_buffer_as_uint16(), dtype=np.uint16).reshape((IMG_HEIGHT, IMG_WIDTH))

            # 推理
            dets, infer_ms = model.infer(frame)
            print(f"[INFO] 推理耗时: {infer_ms:.1f} ms, 检测结果数量: {len(dets)}")

            for det in dets:
                *xyxy, conf, cls = det
                class_id = int(cls)
                print(f"[INFO] 检测到类别ID: {class_id}, 置信度: {conf:.2f}")

                x1, y1, x2, y2 = map(int, np.asarray(xyxy).reshape(-1))
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                depth_val = depth_data[cy, cx]
                dist = pixel_to_horizontal_distance(cx, cy, depth_val, depth_stream)
                if dist is None:
                    continue

                # 阶段1：找物体
                if stage == "FIND_OBJECT" and class_id in TARGET_CLASS_IDS_OBJECT:
                    serh.send("FIND")
                    time.sleep(2)
                    serh.send(f"DIST:{dist:.2f}")
                    print(f"[FSM] 物体锁定，等待 arrive (DIST={dist:.2f}m)")
                    stage = "WAIT_ARRIVE"
                    break

                # 阶段2：找标志物
                elif stage == "FIND_BEGIN" and class_id in TARGET_CLASS_IDS_BEGIN:
                    serh.send("FIND_begin")
                    serh.send(f"DIST_begin:{dist:.2f}")
                    print(f"[FSM] 标志物锁定 (DIST={dist:.2f}m)")
                    stage = "WAIT_FINISH"
                    break

    finally:
        serh.close()
        depth_stream.stop()
        dev.close()
        openni2.unload()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    Grap()
