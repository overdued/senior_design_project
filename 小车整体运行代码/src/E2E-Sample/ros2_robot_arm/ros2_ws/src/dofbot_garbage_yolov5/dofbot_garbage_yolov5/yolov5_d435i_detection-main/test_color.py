import cv2
import numpy as np
from openni import openni2

# OpenNI2 库路径（请根据你的实际路径修改）
OPENNI2_LIB_PATH = "/root/UART_com/camera_location/yolov5_d435i_detection-main/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d/sdk/libs"

# 初始化 OpenNI2
print("Initializing OpenNI2...")
openni2.initialize(OPENNI2_LIB_PATH)

# 打开设备
try:
    dev = openni2.Device.open_any()
    print("Device opened:", dev.get_device_info().name)
except Exception as e:
    print("❌ 无法打开设备:", e)
    exit(1)

# 创建 RGB 流
try:
    color_stream = dev.create_color_stream()
    print("Color stream created.")
except Exception as e:
    print("❌ 无法创建 RGB 流:", e)
    exit(1)

# 启动 RGB 流
try:
    color_stream.start()
    print("Color stream started.")
except Exception as e:
    print("❌ 无法启动 RGB 流:", e)
    exit(1)

# 主循环：读取并显示 RGB 图像
try:
    while True:
        try:
            # 读取 RGB 帧
            color_frame = color_stream.read_frame()
            print("✅ 成功读取 RGB 帧")

            # 转换为 numpy 数组
            color_data = np.frombuffer(color_frame.get_buffer_as_uint8(), dtype=np.uint8)
            color_image = color_data.reshape((color_frame.height, color_frame.width, 3))

            # OpenCV 显示
            cv2.imshow("Color Stream", color_image)

            # 按 'q' 键退出
            if cv2.waitKey(1) == ord('q'):
                break

        except Exception as e:
            print("❌ 读取 RGB 帧失败:", e)
            break

finally:
    # 清理资源
    color_stream.stop()
    dev.close()
    openni2.unload()
    cv2.destroyAllWindows()
    print("✅ 程序结束")