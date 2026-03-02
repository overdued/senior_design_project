import fiftyone.zoo as foz
import fiftyone as fo
import os
from shutil import copy2

# -----------------------------
# 配置目录
# -----------------------------
dataset_dir = "/home/zhaoyuhang/my_dataset_add_cup_cell"
images_dir = os.path.join(dataset_dir, "images/train")
labels_dir = os.path.join(dataset_dir, "labels/train")
os.makedirs(images_dir, exist_ok=True)
os.makedirs(labels_dir, exist_ok=True)

# -----------------------------
# COCO 类别及 class_id
# -----------------------------
classes = ["cup", "cell phone", "remote"]
class_id_map = {"cup": 1, "cell phone": 2, "remote": 3}
samples_per_class = 800

# 已下载图片集合，用来去重
downloaded_images = set()

# -----------------------------
# 下载每类
# -----------------------------
for cls in classes:
    print(f"Downloading COCO class '{cls}' ...")
    dataset = foz.load_zoo_dataset(
        "coco-2017",
        split="train",  # 使用训练集
        label_types=["detections"],
        classes=[cls],
        max_samples=samples_per_class*2,  # 多取一些，防止去重后不足
        shuffle=True,
        dataset_name=f"coco-2017-train-{cls}"  # 每类单独 dataset
    )

    count = 0
    for sample in dataset:
        img_name = os.path.basename(sample.filepath)
        if img_name in downloaded_images:
            continue  # 避免重复图片

        downloaded_images.add(img_name)
        count += 1

        # 图片保存
        dst_img = os.path.join(images_dir, img_name)
        copy2(sample.filepath, dst_img)

        # 生成 YOLOv8 标签，只保留当前类别的框
        lines = []
        if hasattr(sample, "detections") and sample.detections is not None:
            for det in sample.detections.detections:
                if det.label != cls:
                    continue
                x, y, w, h = det.bounding_box  # 已经归一化
                cls_id = class_id_map[cls]
                lines.append(f"{cls_id} {x + w/2:.6f} {y + h/2:.6f} {w:.6f} {h:.6f}\n")

        dst_lbl = os.path.join(labels_dir, img_name.replace(".jpg", ".txt"))
        with open(dst_lbl, "w") as f:
            f.writelines(lines)

        if count >= samples_per_class:
            break  # 达到目标数量

    print(f"Class '{cls}' downloaded: {count} images")

print("✅ COCO dataset download complete!")
print(f"Images: {images_dir}")
print(f"Labels: {labels_dir}")