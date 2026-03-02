from openni import openni2, _openni2 as c_api
import ctypes
#初始化
OPENNI2_LIB_PATH = "/root/UART_com/camera_location/yolov5_d435i_detection-main/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d/sdk/libs"
openni2.initialize(OPENNI2_LIB_PATH)
# 初始化省略...

dev = openni2.Device.open_any()
depth_stream = dev.create_depth_stream()
depth_stream.start()

x, y = 320, 240
frame = depth_stream.read_frame()
frame_data = frame.get_buffer_as_uint16()
depth_array = ctypes.cast(frame_data, ctypes.POINTER(ctypes.c_ushort * (frame.width * frame.height))).contents
depth_val = depth_array[y * frame.width + x]

# 取流句柄
handle = getattr(depth_stream, "_stream", None) or getattr(depth_stream, "_handle")

X = ctypes.c_float()
Y = ctypes.c_float()
Z = ctypes.c_float()

# 调用底层函数（参数要和实际函数匹配）
c_api.CoordinateConverter_convert_depth_to_world(handle,
                                                ctypes.c_int(x),
                                                ctypes.c_int(y),
                                                ctypes.c_ushort(depth_val),
                                                ctypes.byref(X),
                                                ctypes.byref(Y),
                                                ctypes.byref(Z))

print(f"World coords: X={X.value}, Y={Y.value}, Z={Z.value}")
