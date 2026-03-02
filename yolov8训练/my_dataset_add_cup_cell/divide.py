import os
import shutil
import random
from pathlib import Path
import yaml

# =========================
# 路径配置
# =========================
IMG_DIR = "/home/zhaoyuhang/my_dataset_add_cup_cell/train/data_all"
LABEL_DIR = "/home/zhaoyuhang/my_dataset_add_cup_cell/train/labels_all"
OUTPUT_DIR = "/home/zhaoyuhang/my_dataset_add_cup_cell/train_split"

# train/val 比例
VAL_RATIO = 0.1

# 类别配置
NC = 4
NAMES = ["box", "cup", "cell phone", "remote"]

# =========================
# 创建目录
# =========================
IMG_TRAIN = os.path.join(OUTPUT_DIR, "images/train")
IMG_VAL   = os.path.join(OUTPUT_DIR, "images/val")
LABEL_TRAIN = os.path.join(OUTPUT_DIR, "labels/train")
LABEL_VAL   = os.path.join(OUTPUT_DIR, "labels/val")

for d in [IMG_TRAIN, IMG_VAL, LABEL_TRAIN, LABEL_VAL]:
    os.makedirs(d, exist_ok=True)

# =========================
# 列出所有图片
# =========================
all_images = [f for f in os.listdir(IMG_DIR) if f.lower().endswith((".jpg", ".png", ".jpeg"))]
random.shuffle(all_images)

n_total = len(all_images)
n_val = int(n_total * VAL_RATIO)

val_images = set(all_images[:n_val])
train_images = set(all_images[n_val:])

# =========================
# 拷贝图片和标签
# =========================
for img_file in all_images:
    src_img = os.path.join(IMG_DIR, img_file)
    src_label = os.path.join(LABEL_DIR, img_file.replace(".jpg", ".txt"))

    if img_file in train_images:
        dst_img = os.path.join(IMG_TRAIN, img_file)
        dst_label = os.path.join(LABEL_TRAIN, img_file.replace(".jpg", ".txt"))
    else:
        dst_img = os.path.join(IMG_VAL, img_file)
        dst_label = os.path.join(LABEL_VAL, img_file.replace(".jpg", ".txt"))

    shutil.copy(src_img, dst_img)
    if os.path.exists(src_label):
        shutil.copy(src_label, dst_label)

# =========================
# 生成 dataset.yaml
# =========================
dataset_yaml = {
    "train": os.path.abspath(IMG_TRAIN),
    "val": os.path.abspath(IMG_VAL),
    "nc": NC,
    "names": NAMES
}

yaml_path = os.path.join(OUTPUT_DIR, "dataset.yaml")
with open(yaml_path, "w") as f:
    yaml.dump(dataset_yaml, f)

print(f"✅ 数据集划分完成！")
print(f"训练集图片: {len(train_images)} 张")
print(f"验证集图片: {len(val_images)} 张")
print(f"dataset.yaml 已生成: {yaml_path}")