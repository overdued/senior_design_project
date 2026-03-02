import onnx
from onnx import version_converter, helper

# 加载原始 ONNX 模型
model = onnx.load("F:\大四上学期\昇腾升级\yolov8n.onnx")

# 将模型 opset 转换为 12
model = version_converter.convert_version(model, 12)

# 保存新的 ONNX 模型
onnx.save(model, "F:\大四上学期\昇腾升级\yolov8n_opset12.onnx")
