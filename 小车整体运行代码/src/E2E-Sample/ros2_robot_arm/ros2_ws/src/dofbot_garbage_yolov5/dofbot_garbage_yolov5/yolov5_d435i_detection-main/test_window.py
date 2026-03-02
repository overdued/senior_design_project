# import cv2
# import numpy as np

# # 创建一个空白图像
# img = np.zeros((480, 640, 3), dtype=np.uint8)

# # 显示图像
# cv2.imshow("Test Window", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
import cv2
import numpy as np
from openni import openni2

print("Initializing OpenNI...")
openni2.initialize("/root/UART_com/camera_location/yolov5_d435i_detection-main/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d/sdk/libs")
dev = openni2.Device.open_any()
print("Device opened:", dev.get_device_info().name)

depth_stream = dev.create_depth_stream()
color_stream = dev.create_color_stream()
depth_stream.start()
color_stream.start()

try:
    while True:
        print("Reading frames...")
        depth_frame = depth_stream.read_frame()
        # color_frame = color_stream.read_frame()

        depth_data = np.frombuffer(depth_frame.get_buffer_as_uint16(), dtype=np.uint16).reshape((480, 640))
        # color_data = np.frombuffer(color_frame.get_buffer_as_uint8(), dtype=np.uint8).reshape((480, 640, 3))

        print("Displaying frames...")
        cv2.imshow("Color Stream", depth_data)
        if cv2.waitKey(1) == ord('q'):
            break
finally:
    depth_stream.stop()
    # color_stream.stop()
    dev.close()
    openni2.unload()
    cv2.destroyAllWindows()
    print("✅ 程序结束")