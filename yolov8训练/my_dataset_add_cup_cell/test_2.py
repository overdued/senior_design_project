import os
import time
import pandas as pd
from ultralytics import YOLO
import matplotlib.pyplot as plt

# ===================== 配置项 =====================
MODEL_PATHS = {
    "yolov8x_pretrain": "/home/zhaoyuhang/yolov8x.pt",
    "yolo26x": "/home/zhaoyuhang/yolo26x.pt",
    "your_best_model": "/home/zhaoyuhang/runs/detect/train55/weights/best.pt",
    "yolo26n": "/home/zhaoyuhang/yolo26n.pt",
    "yolov8_image7": "/home/zhaoyuhang/yolov8x-oiv7.pt"
}

TEST_IMG_DIR = "/home/zhaoyuhang/test_yolo/data_all"
RESULT_DIR = "/home/zhaoyuhang/test_yolo/result_2"
os.makedirs(RESULT_DIR, exist_ok=True)

# ===================== 原意 -> 数据集类别映射 =====================
CLASS_MAPPING = {
    "box": ["box"],
    "cup": ["coffee cup", "mug","cup"],
    "cell phone": ["mobile phone","cell phone"],
    "remote": ["remote control","remote"]
}

TARGET_CLASSES = list(CLASS_MAPPING.keys())

# ===================== 测试函数 =====================
def test_yolo_models():
    img_files = [f for f in os.listdir(TEST_IMG_DIR) if f.lower().endswith(('.jpg', '.png', '.jpeg'))]
    if not img_files:
        print("❌ 未找到测试图片！")
        return

    summary_data = []

    for model_name, model_path in MODEL_PATHS.items():
        print(f"\n📌 正在测试模型：{model_name}")
        try:
            model = YOLO(model_path)
        except Exception as e:
            print(f"❌ 加载模型 {model_name} 失败：{e}")
            continue

        # 每类计数初始化
        class_counts = {cls: 0 for cls in TARGET_CLASSES}
        total_infer_time = 0.0

        for img_file in img_files:
            img_path = os.path.join(TEST_IMG_DIR, img_file)

            start_time = time.time()
            results = model(img_path, conf=0.5, iou=0.5)
            infer_time = time.time() - start_time
            total_infer_time += infer_time

            result = results[0]
            detected_classes = [model.names[int(box.cls[0])].lower() for box in result.boxes]

            # 按原意统计
            for target_cls, dataset_classes in CLASS_MAPPING.items():
                if any(cls in detected_classes for cls in dataset_classes):
                    class_counts[target_cls] += 1

        avg_time_ms = round((total_infer_time / len(img_files)) * 1000, 2)
        row = {"模型名称": model_name, "平均推理时间(ms)": avg_time_ms, **class_counts}
        summary_data.append(row)

        print(f"✅ {model_name} 测试完成，平均推理时间：{avg_time_ms}ms")
        print(f"  检测统计：{class_counts}")

    # ===================== 保存 CSV =====================
    df = pd.DataFrame(summary_data)
    csv_path = os.path.join(RESULT_DIR, "yolo_class_comparison.csv")
    df.to_csv(csv_path, index=False, encoding="utf-8")
    print(f"\n📊 每类识别统计已保存：{csv_path}")

    # ===================== 可视化柱状图 =====================
    df_plot = df.set_index("模型名称")[TARGET_CLASSES].T

    plt.figure(figsize=(10,6))
    bars = df_plot.plot(kind="bar")

    plt.xlabel("类别", fontsize=12)
    plt.ylabel("识别图片数量", fontsize=12)
    plt.title("各模型按类别识别能力对比", fontsize=14)
    plt.xticks(rotation=0)
    plt.grid(axis="y", alpha=0.3)

    plt.tight_layout()  # 确保标签不被截断
    plot_path = os.path.join(RESULT_DIR, "model_class_comparison_2.png")
    plt.savefig(plot_path, dpi=300)
    plt.close()  # 关闭图，防止后续绘图干扰
    print(f"📈 对比柱状图已保存：{plot_path}")


if __name__ == "__main__":
    test_yolo_models()