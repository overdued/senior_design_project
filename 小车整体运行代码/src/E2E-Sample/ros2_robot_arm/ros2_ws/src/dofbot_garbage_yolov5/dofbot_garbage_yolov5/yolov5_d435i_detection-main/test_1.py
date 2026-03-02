from openni import openni2
import numpy as np
import cv2
import os
import sys
import time
import ctypes

OPENNI2_LIB_PATH = "/root/UART_com/camera_location/yolov5_d435i_detection-main/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d/sdk/libs"
openni2.initialize(OPENNI2_LIB_PATH)

OUTPUT_DIR = "/root/UART_com/camera_location/yolov5_d435i_detection-main/cameras"
os.makedirs(OUTPUT_DIR, exist_ok=True)

clicked_points = []

if __name__ == "__main__":
    try:
        dev = openni2.Device.open_any()
    except Exception as e:
        print("无法打开设备:", e)
        sys.exit(1)

    # 创建深度和彩色流
    depth_stream = dev.create_depth_stream()
    color_stream = dev.create_color_stream()

    # 设置图像配准，使深度和彩色图像对齐
    dev.set_image_registration_mode(True)

    depth_stream.start()
    color_stream.start()

    try:
        while True:
            # 读取深度帧数据
            frame = depth_stream.read_frame()
            frame_data = frame.get_buffer_as_uint16()
            depth_array = np.frombuffer(frame_data, dtype=np.uint16).reshape((frame.height, frame.width))

            # 选定测试像素点
            x, y = 320, 240
            depth_val = depth_array[y, x]

            # 1. 深度坐标 + 深度值 转换为 世界坐标 (单位：米)
            X, Y, Z = openni2.convert_depth_to_world(depth_stream, x, y, depth_val)

            # 2. 世界坐标 转换回 深度坐标（像素坐标 + 深度值）
            depth_x, depth_y, depth_z = openni2.convert_world_to_depth(depth_stream, X, Y, Z)

            # 3. 深度坐标 转换到 彩色图像坐标，注意传入深度和彩色流
            color_x = ctypes.c_int()
            color_y = ctypes.c_int()
            #openni2.convert_depth_to_color(depth_stream, color_stream, x, y, depth_val)

            print(f"像素点 ({x},{y}) 深度值: {depth_val} mm")
            print(f"转换为世界坐标: X={X:.3f} m, Y={Y:.3f} m, Z={Z:.3f} m")
            print(f"世界坐标转换回深度坐标: x={depth_x:.3f}, y={depth_y:.3f}, depth={depth_z:.3f}")
            print(f"对应彩色图像坐标: x={color_x.value}, y={color_y.value}")

            # 记录数据
            clicked_points.append((time.time(), x, y, depth_val, X, Y, Z, depth_x, depth_y, depth_z, color_x.value, color_y.value))

            # 伪彩色显示深度图并标注点
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_array, alpha=0.03), cv2.COLORMAP_JET)
            cv2.circle(depth_colormap, (x, y), 5, (0, 255, 0), 2)
            # cv2.imshow("Depth View", depth_colormap)

            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

            # time.sleep(1)

    except KeyboardInterrupt:
        print("采集结束")

    depth_stream.stop()
    color_stream.stop()
    dev.close()
    openni2.unload()
    cv2.destroyAllWindows()

    # 保存 CSV 文件
    csv_path = os.path.join(OUTPUT_DIR, "converted_points.csv")
    import csv
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Timestamp", "Depth_X", "Depth_Y", "Depth_Value(mm)",
            "World_X(m)", "World_Y(m)", "World_Z(m)",
            "Converted_Depth_X", "Converted_Depth_Y", "Converted_Depth_Value",
            "Color_X", "Color_Y"
        ])
        writer.writerows(clicked_points)

    print(f"数据已保存至 {csv_path}")
