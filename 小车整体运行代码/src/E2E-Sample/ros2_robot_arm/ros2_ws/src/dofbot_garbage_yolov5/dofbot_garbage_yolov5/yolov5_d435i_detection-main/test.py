from openni import openni2, _openni2 as c_api
import numpy as np
import cv2
import os
import csv
import sys

# 初始化 OpenNI2，指定 .so 路径（视你的安装位置而定）
OPENNI2_LIB_PATH = "/root/UART_com/camera_location/yolov5_d435i_detection-main/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d/sdk/libs"  # 或你的实际安装路径，如 /usr/local/lib
openni2.initialize(OPENNI2_LIB_PATH)  # 加载 OpenNI2 .so 文件

# 保存点击的坐标和标注
clicked_points = []
annotations = []

# 鼠标回调函数：双击显示坐标
def mousecallback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        depth_val = dpt[y, x]
        print(f"像素坐标: ({x}, {y})，深度: {depth_val:.2f} mm")

        # 计算三维坐标
        depth_stream = param
        X, Y, Z = c_api.CoordinateConverter_convert_depth_to_world(depth_stream._stream, x, y, depth_val)
        print(f"三维坐标: X={X:.2f} mm, Y={Y:.2f} mm, Z={Z:.2f} mm")

        # 添加到列表中
        clicked_points.append((x, y, depth_val, X, Y, Z))

        # 标注文字
        label = f"({int(X)}, {int(Y)}, {int(Z)})"
        annotations.append((x, y, label))

# 主程序
if __name__ == "__main__":
    try:
        dev = openni2.Device.open_any()
    except Exception as e:
        print("❌ 无法打开 OpenNI2 设备，请确认设备已连接并驱动正常:", e)
        sys.exit(1)

    print("设备信息:", dev.get_device_info())

    depth_stream = dev.create_depth_stream()
    dev.set_image_registration_mode(True)
    depth_stream.start()

    # 摄像头可能不是 /dev/video0，检查实际连接
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("⚠️ 无法打开摄像头，请检查 /dev/video* 是否存在")
    
    cv2.namedWindow('depth')
    cv2.setMouseCallback('depth', mousecallback, depth_stream)

    while True:
        frame = depth_stream.read_frame()
        dframe_data = np.array(frame.get_buffer_as_triplet()).reshape([480, 640, 2])
        dpt1 = np.asarray(dframe_data[:, :, 0], dtype='float32')
        dpt2 = np.asarray(dframe_data[:, :, 1], dtype='float32')
        dpt2 *= 255
        dpt = dpt1 + dpt2

        # 生成伪彩色深度图
        dim_gray = cv2.convertScaleAbs(dpt, alpha=0.17)
        depth_colormap = cv2.applyColorMap(dim_gray, 2)

        # 显示标注在图上
        for (x, y, label) in annotations:
            cv2.circle(depth_colormap, (x, y), 4, (0, 255, 255), -1)  # 画点
            cv2.putText(depth_colormap, label, (x + 5, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow('depth', depth_colormap)

        # 如果有摄像头，则显示彩色图
        if cap.isOpened():
            ret, color_frame = cap.read()
            if ret:
                cv2.imshow('color', color_frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    # 清理资源
    depth_stream.stop()
    cap.release()
    cv2.destroyAllWindows()
    dev.close()

    # 保存点击记录
    with open("clicked_points.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Pixel_X", "Pixel_Y", "Depth(mm)", "World_X(mm)", "World_Y(mm)", "World_Z(mm)"])
        writer.writerows(clicked_points)

    print("✅ 坐标数据已保存到 clicked_points.csv")
