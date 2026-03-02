import os
import time
import csv
import cv2
import numpy as np
import math
import torch
from openni import openni2
from utils.torch_utils import time_sync
from utils.general import check_img_size, non_max_suppression, scale_coords
from utils.datasets import letterbox
from models.experimental import attempt_load

# ========== 配置 ==========
YOLOV5_WEIGHTS = "/root/UART_com/camera_location/yolov5_d435i_detection-main/weights/yolov5s.pt"
YOLO_INPUT_SIZE = 640
OUTPUT_CSV = "cameras/fused_result.csv"
FOCAL_LENGTH = 740
IMG_WIDTH = 640
IMG_HEIGHT = 480

# 👉 添加几何测距参数
CAMERA_HEIGHT = 1.2  # 米
CAMERA_PITCH_DEG = 30  # 摄像头俯角，单位：度
CAMERA_PITCH_RAD = math.radians(CAMERA_PITCH_DEG)

MIN_DEPTH_VALID = 450  # mm，小于这个值就不信任深度数据

# ========== 初始化设备 ==========
openni2.initialize("/root/UART_com/camera_location/yolov5_d435i_detection-main/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d/sdk/libs")
dev = openni2.Device.open_any()
depth_stream = dev.create_depth_stream()
depth_stream.start()

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ 无法打开摄像头")
    exit(1)

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = attempt_load(YOLOV5_WEIGHTS, map_location=device)
stride = int(model.stride.max())
img_size = check_img_size(YOLO_INPUT_SIZE, s=stride)
model.eval()

def preprocess(img):
    img0 = img.copy()
    img = letterbox(img0, new_shape=img_size, auto=False)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device).float() / 255.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    return img, img0

clicked_points = []

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 图像获取失败")
            break

        depth_frame = depth_stream.read_frame()
        depth_data = np.frombuffer(depth_frame.get_buffer_as_uint16(), dtype=np.uint16).reshape((480, 640))

        img_tensor, canvas = preprocess(frame)
        with torch.no_grad():
            pred = model(img_tensor, augment=False)[0]
        pred = non_max_suppression(pred, 0.25, 0.45)

        for det in pred:
            if det is not None and len(det):
                det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], canvas.shape).round()
                for *xyxy, conf, cls in det:
                    x1, y1, x2, y2 = map(int, xyxy)
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    w = x2 - x1
                    h = y2 - y1
                    class_id = int(cls)

                    depth_val = depth_data[cy, cx]
                    Z, X, Y = None, None, None

                    # ✅ 情况1：优先使用深度相机
                    if depth_val > 0 and depth_val >= MIN_DEPTH_VALID:
                        X, Y, Z = openni2.convert_depth_to_world(depth_stream, cx, cy, depth_val)
                        X, Y, Z = round(X / 1000, 3), round(Y / 1000, 3), round(Z / 1000, 3)
                        print(f"✅ 深度测距 Z={Z} X={X} Y={Y}")

                    # ✅ 情况2：深度不可用，使用几何估距
                    else:
                        dy = cy - (IMG_HEIGHT / 2)
                        theta_pixel = math.atan2(dy, FOCAL_LENGTH)
                        total_angle = CAMERA_PITCH_RAD + theta_pixel
                        if abs(math.tan(total_angle)) > 1e-3:
                            Z = round(CAMERA_HEIGHT / math.tan(total_angle), 3)
                            X = round((cx - IMG_WIDTH / 2) * Z / FOCAL_LENGTH, 3)
                            Y = 0.0
                            print(f"⚠️ 几何估距 Z={Z} X={X}")
                        else:
                            Z = None

                    # ✅ 可视化
                    label = f"{class_id} {conf:.2f}"
                    cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(canvas, (cx, cy), 4, (255, 255, 255), -1)
                    if Z:
                        cv2.putText(canvas, f"XYZ:({X},{Y},{Z})", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                    clicked_points.append([time.time(), cx, cy, depth_val, X, Y, Z])

        cv2.imshow("YOLOv5 Fused Estimation", canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    depth_stream.stop()
    dev.close()
    openni2.unload()
    cv2.destroyAllWindows()

    os.makedirs(os.path.dirname(OUTPUT_CSV), exist_ok=True)
    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Pixel_X", "Pixel_Y", "Depth(mm)", "World_X(m)", "World_Y(m)", "World_Z(m)"])
        writer.writerows(clicked_points)

    print(f"✅ 数据保存到 {OUTPUT_CSV}")
