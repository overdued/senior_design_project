#!/usr/bin/env python3
import time
import sys

from dofbot_garbage_yolov5.test import YOLO_OI601

MODEL = "/root/ascend-devkit/src/E2E-Sample/ros2_robot_arm/ros2_ws/src/dofbot_garbage_yolov5/dofbot_garbage_yolov5/yolov8x_32.om"
CLASS_PATH = "/root/ascend-devkit/src/E2E-Sample/ros2_robot_arm/ros2_ws/src/dofbot_garbage_yolov5/dofbot_garbage_yolov5/classes.txt"

def main():
    print("🧠 Step 1: 创建 YOLO 模型（位置参数）")

    # ⚠️ 关键修改：不要用关键字参数
    model = YOLO_OI601(
        MODEL,
        CLASS_PATH,
        0.25,   # conf_thr
        0.5     # iou_thr
    )

    print("✅ 模型创建完成，休眠 5 秒")
    time.sleep(5)

    print("🧹 Step 2: 释放 NPU 资源")
    model.session.free_resource()

    print("🎉 OK：最小 YOLO 初始化 + 释放 测试完成")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("❌ 发生异常：", e)
        sys.exit(1)
