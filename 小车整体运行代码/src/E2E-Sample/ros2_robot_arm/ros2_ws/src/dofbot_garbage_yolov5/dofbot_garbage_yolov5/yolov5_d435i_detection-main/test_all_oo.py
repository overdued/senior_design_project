import os
import time
import csv
import cv2
import numpy as np
import torch
from utils.torch_utils import select_device, time_sync
from utils.general import check_img_size, non_max_suppression, scale_coords
from utils.datasets import letterbox
from models.experimental import attempt_load

# ========== 配置路径 ==========
YOLOV5_WEIGHTS = "/root/UART_com/camera_location/yolov5_d435i_detection-main/weights/yolov5s.pt"  # YOLO 权重路径
YOLO_INPUT_SIZE = 640  # 输入图像大小
OUTPUT_CSV = "cameras/output.csv"  # 输出 CSV 文件

# ========== 初始化摄像头 ==========
cap = cv2.VideoCapture(0)  # 使用默认摄像头（/dev/video0）
if not cap.isOpened():
    print("❌ 无法打开摄像头")
    exit(1)

# ========== 初始化 YOLOv5 ==========
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = attempt_load(YOLOV5_WEIGHTS, map_location=device)
stride = int(model.stride.max())
img_size = check_img_size(YOLO_INPUT_SIZE, s=stride)
model.eval()

# ========== 图像预处理函数 ==========
def preprocess(img):
    img0 = img.copy()
    img = letterbox(img0, new_shape=img_size, auto=False)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, HWC to CHW
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device).float()
    img /= 255.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    return img, img0

# ========== 主程序 ==========
clicked_points = []
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 无法读取图像帧")
            break

        # 目标检测
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

                    # 可视化
                    label = f"{int(cls)} {conf:.2f}"
                    cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(canvas, (cx, cy), 4, (255, 255, 255), -1)
                    cv2.putText(canvas, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                    # 记录数据（无深度信息）
                    clicked_points.append([time.time(), cx, cy, None, None, None, None])

        # 显示结果
        cv2.imshow("Detection", canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()

    # 保存结果
    os.makedirs(os.path.dirname(OUTPUT_CSV), exist_ok=True)
    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Pixel_X", "Pixel_Y", "Depth(mm)", "World_X(m)", "World_Y(m)", "World_Z(m)"])
        writer.writerows(clicked_points)
    print(f"✅ 数据保存到 {OUTPUT_CSV}")