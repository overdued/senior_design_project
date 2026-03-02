import os
import time
import csv
import cv2
import numpy as np
from openni import openni2
import torch
from utils.torch_utils import time_sync
from utils.general import check_img_size, non_max_suppression, scale_coords
from utils.datasets import letterbox
from models.experimental import attempt_load

# ========== 配置路径 ==========
YOLOV5_WEIGHTS = "/root/UART_com/camera_location/yolov5_d435i_detection-main/weights/yolov5s.pt"
YOLO_INPUT_SIZE = 640
OUTPUT_CSV = "cameras/fused_output.csv"
OPENNI2_LIB_PATH = "/root/UART_com/camera_location/yolov5_d435i_detection-main/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d/sdk/libs"

# ========== 初始化设备 ==========
# 初始化深度相机
openni2.initialize(OPENNI2_LIB_PATH)
dev = openni2.Device.open_any()
depth_stream = dev.create_depth_stream()
depth_stream.start()

# 初始化 RGB 摄像头
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ 无法打开 RGB 摄像头")
    exit(1)

# 初始化 YOLOv5
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = attempt_load(YOLOV5_WEIGHTS, map_location=device)
stride = int(model.stride.max())
img_size = check_img_size(YOLO_INPUT_SIZE, s=stride)
model.eval()

# 图像预处理
def preprocess(img):
    img0 = img.copy()
    img = letterbox(img0, new_shape=img_size, auto=False)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device).float() / 255.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    return img, img0

# 主循环
clicked_points = []
try:
    while True:
        # 获取 RGB 图像
        ret, frame = cap.read()
        if not ret:
            print("❌ 无法读取 RGB 图像帧")
            break

        # 获取深度图像
        depth_frame = depth_stream.read_frame()
        depth_data = np.frombuffer(depth_frame.get_buffer_as_uint16(), dtype=np.uint16).reshape((480, 640))

        # YOLO 检测（只在 RGB 图像）
        img_tensor, canvas = preprocess(frame)
        t1 = time_sync()
        with torch.no_grad():
            pred = model(img_tensor, augment=False)[0]
        pred = non_max_suppression(pred, 0.25, 0.45)
        t2 = time_sync()

        # 解析检测结果
        for det in pred:
            if det is not None and len(det):
                det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], canvas.shape).round()
                for *xyxy, conf, cls in det:
                    x1, y1, x2, y2 = map(int, xyxy)
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    if 0 <= cx < 640 and 0 <= cy < 480:
                        depth_val = depth_data[cy, cx]
                        if depth_val == 0:
                            continue
                        X, Y, Z = openni2.convert_depth_to_world(depth_stream, cx, cy, depth_val)
                        X, Y, Z = round(X / 1000, 3), round(Y / 1000, 3), round(Z / 1000, 3)

                        # 显示
                        label = f"{int(cls)} {conf:.2f}"
                        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(canvas, (cx, cy), 4, (255, 255, 255), -1)
                        cv2.putText(canvas, f"XYZ:({X},{Y},{Z})", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                        clicked_points.append([time.time(), cx, cy, depth_val, X, Y, Z])

        # 显示图像
        cv2.imshow("YOLOv5 Detection", canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    depth_stream.stop()
    dev.close()
    openni2.unload()
    cv2.destroyAllWindows()

    # 保存 CSV
    os.makedirs(os.path.dirname(OUTPUT_CSV), exist_ok=True)
    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Pixel_X", "Pixel_Y", "Depth(mm)", "World_X(m)", "World_Y(m)", "World_Z(m)"])
        writer.writerows(clicked_points)

    print(f"✅ 数据保存到 {OUTPUT_CSV}")
