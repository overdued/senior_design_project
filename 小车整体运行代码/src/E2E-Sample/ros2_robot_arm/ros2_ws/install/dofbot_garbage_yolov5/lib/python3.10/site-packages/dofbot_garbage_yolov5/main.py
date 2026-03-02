import Arm_Lib
import cv2 as cv
from pathlib import Path
import numpy as np
import serial
import time
from time import sleep
import os
import math
import traceback
from openni import openni2
from .test import YOLO_OI601
from .utils.dofbot_config import ArmCalibration, read_XYT
from .utils.garbage_identify import GarbageIdentify
import socket
from typing import Callable, Optional, Tuple

# 几何测距参数
FOCAL_LENGTH = 740
IMG_WIDTH = 640
IMG_HEIGHT = 480

CAMERA_HEIGHT = 0.1  # 米
CAMERA_PITCH_DEG = 14
CAMERA_PITCH_RAD = math.radians(CAMERA_PITCH_DEG)
MIN_DEPTH_VALID = 450  # mm，小于此值认为不可用

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

def wait_for_message(
    host: str = "0.0.0.0",
    port: int = 5009,
    match: Optional[Callable[[bytes], bool]] = None,
    timeout: Optional[float] = None,
    backlog: int = 1,
    max_read: int = 1024 * 1024,  # 1MB 安全上限，避免恶意大包
) -> Tuple[bytes, Tuple[str, int]]:
    """
    启动一次性 TCP 监听，直到收到满足 match 条件的数据后返回。
    返回 (data, addr)。无 match 时，收到任意非空数据即返回。

    - host: 监听地址，例如 "0.0.0.0" 或具体网卡 "192.168.1.100"
    - port: 端口
    - match: 匹配函数，入参为 bytes，返回 True 表示满足条件
    - timeout: socket 超时（秒），包含 accept 和 recv
    - backlog: listen 的排队长度
    - max_read: 最大累计读取字节数（防御性）
    """
    if match is None:
        match = lambda b: len(b) > 0

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen(backlog)
        if timeout is not None:
            s.settimeout(timeout)

        print(f"[wait_for_message] Listening on {host}:{port} ...")
        try:
            conn, addr = s.accept()  # 仅接受一次连接
        except socket.timeout:
            raise TimeoutError(f"Accept timeout after {timeout}s on {host}:{port}")

        with conn:
            print(f"[wait_for_message] Accepted connection from {addr}")
            if timeout is not None:
                conn.settimeout(timeout)

            buf = bytearray()
            while True:
                try:
                    chunk = conn.recv(4096)
                    text = chunk.decode("utf-8", errors="replace")
                    conn.sendall(b"OK\n")
                except socket.timeout:
                    raise TimeoutError(f"Recv timeout after {timeout}s from {addr}")

                if not chunk:
                    # 对端关闭连接
                    print("[wait_for_message] Peer closed connection")
                    break

                buf += chunk
                if len(buf) > max_read:
                    raise ValueError(f"Exceeded max_read={max_read} bytes")

                # 如果当前缓冲已满足条件，则返回
                result, keyword = match(text)
                if result:
                    print("[wait_for_message] Match satisfied, returning")
                    return keyword, addr
                
def hand_grasp():
    ser.write(data2_hex)
    
def hand_open():
    ser.write(data1_hex)
    
def hand_default():
    ser.write(data3_hex)

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

def search_and_grasp_target(target_object_name="Plastic bag"):
    """
    搜索并抓取指定目标物体
    :param target_object_name: 要抓取的目标物体名称，默认为 "Plastic bag"
    :return: True if successful, False otherwise
    """
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
    Grapfinishstate = False
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
    
    # 识别逻辑控制变量
    current_angle = 90  # 当前角度
    angle_step = 10     # 角度步进
    min_angle = 30      # 最小角度
    max_angle = 120     # 最大角度
    
    # 识别逻辑控制变量
    phase = 1  # 1: 90度测试阶段, 2: 全角度遍历阶段
    detection_results = []  # 存储检测结果 [(angle, is_target_found, is_in_center), ...]
    max_detections_per_angle = 10  # 每个角度最大检测次数
    target_detection_threshold = 1  # 需要成功检测到目标的次数
    
    # 中心区域检测参数
    center_ratio = 0.6  # 中心区域占图像的比例
    
    # XYT参数路径
    XYT_path = os.path.join(cfg_folder, "XYT_config.txt")
    try:
        xy, _ = read_XYT(XYT_path)
    except Exception:
        print("Read XYT_config Error !!!")
        return False

    print("Read xy is", xy)

    joints_0 = [xy[0], xy[1], 0, 0, 135, 30]
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
    
    success = False
    current_round = 1
    
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened() and not Grapfinishstate:
        print(f"========== 开始第 {current_round} 轮搜索 ==========")
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
                dp_data = np.fromfile(dp_cfg_path, dtype=np.int32)
                
                # 修复reshape问题：检查dp的大小并正确处理
                if len(dp_data) >= 8:
                    # 取前8个元素reshape为4x2
                    dp_temp = dp_data[:8].reshape(4, 2)
                    # 转换为正确的格式，确保是float32类型
                    dp_points = []
                    for i in range(4):
                        dp_points.append([float(dp_temp[i][0]), float(dp_temp[i][1])])
                    dp = np.array(dp_points, dtype=np.float32)
                    
                    # 执行透视变换
                    img = calibration.perspective_transform(dp, img)
                else:
                    # 如果数据不足，使用原始图像
                    print("透视变换数据不足，使用原始图像")
            else:
                # 如果文件不存在，使用原始图像
                print("dp.bin文件不存在，使用原始图像")

        except Exception as e:
            print(f"Perspective transform error: {e}")
            print("Using original image without perspective transform")

        # 根据当前阶段执行不同的逻辑
        if phase == 1:
            # 阶段1: 在90度位置测试
            print(f"阶段1: 在90度位置进行测试 (第{len([r for r in detection_results if r[0]==90])+1}次)")
            
            # 移动机械臂到90度
            arm.Arm_serial_servo_write(1, 90, 500)

            # 重置检测状态
            msg = {}
            if hasattr(target, 'last_detection_coords'):
                target.last_detection_coords = {}

            # 执行识别
            try:
                img, msg = target.garbage_run(img)
                
                # 检查是否识别到目标物体
                target_found = False
                box_in_center = False
                box_coords = None
                
                print(f"msg is: {msg}")
                
                if len(msg) != 0:
                    object_names = list(msg.keys())
                    for name in object_names:
                        # 严格匹配目标物体名称
                        if name.lower() == target_object_name.lower():
                            target_found = True
                            Finddegree = 90
                            print(f"找到目标物体 {target_object_name}，角度:", Finddegree)
                            
                            # 检查是否在中心区域
                            if hasattr(target, 'last_detection_coords') and name in target.last_detection_coords:
                                x1, y1, x2, y2 = target.last_detection_coords[name]
                                box_coords = (x1, y1, x2, y2)
                                box_in_center = is_box_in_center_region(x1, y1, x2, y2, 640, 480, center_ratio)
                                print(f"Box坐标: ({x1}, {y1}, {x2}, {y2})")
                                print(f"Box是否在中心区域: {box_in_center}")
                            else:
                                box_in_center = True
                                print("没有坐标信息，默认认为在中心")
                            break
                else:
                    print("未检测到任何目标")
                
                # 只有真正检测到目标且在中心才记录为成功
                detection_results.append((90, target_found, box_in_center))
                
                # 统计在90度的检测结果
                angle_90_results = [r for r in detection_results if r[0] == 90]
                successful_detections = sum(1 for r in angle_90_results if r[1] and r[2])  # 目标存在且在中心
                total_detections = len(angle_90_results)
                
                print(f"90度检测结果: 成功{successful_detections}次, 总共{total_detections}次")
                
                # 判断是否满足条件
                if total_detections >= max_detections_per_angle:
                    if successful_detections >= target_detection_threshold and target_found:
                        print(f"在90度位置成功检测到目标 {target_object_name} 且在中心区域，执行抓取...")
                        target.garbage_grap(msg, xy, 90)
                        success = True
                        Grapfinishstate = True
                        break
                    else:
                        print("在90度位置未满足抓取条件，进入阶段2...")
                        phase = 2
                        current_angle = min_angle
                        detection_results = []  # 清空之前的检测结果
                        
            except Exception as e:
                print(f"识别过程中出现错误: {e}")
                traceback.print_exc()
                detection_results.append((90, False, False))
                
                # 检查是否达到最大检测次数
                angle_90_results = [r for r in detection_results if r[0] == 90]
                if len(angle_90_results) >= max_detections_per_angle:
                    print("在90度位置检测次数已达上限，进入阶段2...")
                    phase = 2
                    current_angle = min_angle
                    detection_results = []
                
        elif phase == 2:
            # 阶段2: 从30度开始遍历所有角度
            print(f"阶段2: 在{current_angle}度位置进行检测")
            
            # 移动机械臂到当前角度
            arm.Arm_serial_servo_write(1, current_angle, 500)

            # 重置检测状态
            msg = {}
            if hasattr(target, 'last_detection_coords'):
                target.last_detection_coords = {}

            # 执行识别
            try:
                img, msg = target.garbage_run(img)
                
                # 检查是否识别到目标物体
                target_found = False
                box_in_center = False
                
                print(f"msg is: {msg}")
                
                if len(msg) != 0:
                    object_names = list(msg.keys())
                    for name in object_names:
                        # 严格匹配目标物体名称
                        if name.lower() == target_object_name.lower():
                            target_found = True
                            Finddegree = current_angle
                            print(f"找到目标物体 {target_object_name}，角度:", Finddegree)
                            
                            # 检查是否在中心区域
                            if hasattr(target, 'last_detection_coords') and name in target.last_detection_coords:
                                x1, y1, x2, y2 = target.last_detection_coords[name]
                                box_in_center = is_box_in_center_region(x1, y1, x2, y2, 640, 480, center_ratio)
                                print(f"Box坐标: ({x1}, {y1}, {x2}, {y2})")
                                print(f"Box是否在中心区域: {box_in_center}")
                            else:
                                box_in_center = True
                                print("没有坐标信息，默认认为在中心")
                            break
                else:
                    print("未检测到任何目标")
                
                # 只有真正检测到目标且在中心才记录为成功
                detection_results.append((current_angle, target_found, box_in_center))
                
                # 统计当前角度的检测结果
                current_angle_results = [r for r in detection_results if r[0] == current_angle]
                successful_detections = sum(1 for r in current_angle_results if r[1] and r[2])
                total_detections = len(current_angle_results)
                
                print(f"{current_angle}度检测结果: 成功{successful_detections}次, 总共{total_detections}次")
                
                # 判断是否满足条件或达到最大检测次数
                if total_detections >= max_detections_per_angle:
                    if successful_detections >= target_detection_threshold and target_found:
                        print(f"在{current_angle}度位置成功检测到目标 {target_object_name} 且在中心区域，执行抓取...")
                        target.garbage_grap(msg, xy, current_angle)
                        success = True
                        Grapfinishstate = True
                        break
                    else:
                        print(f"在{current_angle}度位置未满足抓取条件，切换到下一个角度...")
                        # 切换到下一个角度
                        current_angle += angle_step
                        if current_angle > max_angle:
                            print("已完成所有角度的搜索，未找到合适目标，重新开始新一轮搜索...")
                            # 开始下一轮搜索
                            current_round += 1
                            print(f"开始第 {current_round} 轮搜索...")
                            # 重置搜索参数，直接进入阶段2（从30度开始）
                            phase = 2
                            current_angle = min_angle
                            detection_results = []
                        detection_results = []  # 清空当前角度的检测结果
                        
                # 如果检测到目标但没有抓取，继续寻找下一个角度
                elif target_found and not box_in_center:
                    print(f"在{current_angle}度位置检测到目标但不在中心区域，切换到下一个角度...")
                    current_angle += angle_step
                    if current_angle > max_angle:
                        print("已完成所有角度的搜索，未找到合适目标，重新开始新一轮搜索...")
                        current_round += 1
                        print(f"开始第 {current_round} 轮搜索...")
                        phase = 2
                        current_angle = min_angle
                        detection_results = []
                    detection_results = []  # 清空当前角度的检测结果
                        
            except Exception as e:
                print(f"识别过程中出现错误: {e}")
                traceback.print_exc()
                detection_results.append((current_angle, False, False))
                
                # 检查是否达到最大检测次数
                current_angle_results = [r for r in detection_results if r[0] == current_angle]
                if len(current_angle_results) >= max_detections_per_angle:
                    print(f"在{current_angle}度位置检测次数已达上限，切换到下一个角度...")
                    current_angle += angle_step
                    if current_angle > max_angle:
                        print("已完成所有角度的搜索，未找到合适目标，重新开始新一轮搜索...")
                        current_round += 1
                        print(f"开始第 {current_round} 轮搜索...")
                        phase = 2
                        current_angle = min_angle
                        detection_results = []
                    detection_results = []  # 清空当前角度的检测结果
    
    # 释放资源
    try:
        if 'capture' in locals() and capture.isOpened():
            capture.release()
            print("机械臂摄像头释放")
        cv.destroyAllWindows()
        target.model.free_resource()
        print("机械臂模型释放")
        ser.close()
    except:
        pass
    
    if success:
        print(f"成功抓取目标物体: {target_object_name}")
    else:
        print(f"未能成功抓取目标物体: {target_object_name}")
    
    return success

# 使用示例：
def Grap(text):
	
    search_and_grasp_target(text)  # 抓取塑料袋
    return



# ========== 配置 ==========
MODEL = "/root/ascend-devkit/src/E2E-Sample/ros2_robot_arm/ros2_ws/src/dofbot_garbage_yolov5/dofbot_garbage_yolov5/yolov8x_32.om"
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
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 模型初始化
    print("加载 YOLO 模型中...")
    model = YOLO_OI601(MODEL, CLASS_PATH, conf_thr=0.25, iou_thr=0.5)
    print("? 模型加载成功")

    # keyword = {"蛋糕", "盒子"}
    
    # def match_ready(data) -> bool:
    #     for kw in keyword:
    #         if kw in data:
    #             return True, kw
    #     return False, None
            
    # text, addr = wait_for_message(
    #             host="192.168.1.100",  
    #             port=5009,
    #             match=match_ready,
    #             timeout=240.0,         
    #             backlog=1,
    #             max_read=2 * 1024 * 1024,
    #         )

    # # 根据app结果决定小车目标和机械臂目标
    # if text == "蛋糕":
    #     TARGET_CLASS_IDS_OBJECT = {394}
    #     target_arm = "Plastic bag"
    # elif text == "盒子":
    #     TARGET_CLASS_IDS_OBJECT = {62}
    #     target_arm = "Box"
    
    TARGET_CLASS_IDS_OBJECT = {62}
    target_arm = "Box"
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
                    cap.release()
                    print("小车摄像头释放")
                    model.session.free_resource()
                    print("小车模型卸载")
                    Grap(target_arm)
                    model = YOLO_OI601(MODEL, CLASS_PATH, conf_thr=0.25, iou_thr=0.5)
                    print("小车模型加载")
                    cap = cv.VideoCapture(0)
                    print("小车摄像头加载")
                    Grabfinishstate=False
                    sleep(1)
                    
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
                # 几何测距
                dy = cy - (IMG_HEIGHT / 2)
                theta_pixel = math.atan2(dy, FOCAL_LENGTH)
                total_angle = CAMERA_PITCH_RAD + theta_pixel
                Z_g = round(CAMERA_HEIGHT / math.tan(total_angle), 3)
                X_g = round((cx - IMG_WIDTH / 2) * Z_g / FOCAL_LENGTH, 3)
                Y_g = 0.0
                geom_Z = Z_g
                try:
                    Z_horizontal2 = round(math.sqrt(geom_Z**2 - CAMERA_HEIGHT**2), 3)
                    print(f"几何测距Z={Z_horizontal2:.3f} X={X_g:.3f}")
                except:
                    pass
                # 深度测距
                depth_val = depth_data[cy, cx]
                X_d, Y_d, Z_d = openni2.convert_depth_to_world(depth_stream, cx, cy, depth_val)
                X_d, Y_d, Z_d = round(X_d / 1000, 3), round(Y_d / 1000, 3), round(Z_d / 1000, 3)
                depth_Z = Z_d
                try:
                    Z_horizontal1 = round(math.sqrt(depth_Z**2 - CAMERA_HEIGHT**2), 3)
                    print(f"? 深度测距：Z={Z_horizontal1:.3f} X={X_d:.3f} Y={Y_d:.3f}")
                except:
                    pass

                # 阶段1：找物体
                if stage == "FIND_OBJECT" and class_id in TARGET_CLASS_IDS_OBJECT:
                    serh.send("FIND")
                    time.sleep(2)
                    serh.send(f"DIST:{Z_horizontal2:.2f}")
                    print(f"[FSM] 物体锁定，等待 arrive (DIST={Z_horizontal2:.2f}m)")
                    stage = "WAIT_ARRIVE"
                    break

                # 阶段2：找标志物
                elif stage == "FIND_BEGIN" and class_id in TARGET_CLASS_IDS_BEGIN:
                    serh.send("FIND_begin")
                    serh.send(f"DIST:{Z_horizontal1:.2f}")      
                    print(f"[FSM] 标志物锁定 (DIST={Z_horizontal1:.2f}m)")
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
    main()