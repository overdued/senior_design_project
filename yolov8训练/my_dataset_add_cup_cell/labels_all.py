import json
import os
from tqdm import tqdm
import shutil

# ========= 路径配置 =========
# COCO JSON 和图片路径（只包含 cup/cell phone/remote）
COCO_JSON = "/home/zhaoyuhang/fiftyone/coco-2017/raw/instances_train2017.json"
COCO_IMG_DIR = "/home/zhaoyuhang/my_dataset_add_cup_cell/images/train"       # COCO 测试图片
COCO_LABEL_DIR = "/home/zhaoyuhang/my_dataset_add_cup_cell/labels/train"   # YOLO txt 输出路径

# box 图片和 labels（已经标好 0）
BOX_IMG_DIR = "/home/zhaoyuhang/my_dataset/images/train"
BOX_LABEL_DIR = "/home/zhaoyuhang/my_dataset/labels/train"

# 最终测试集存放路径
TEST_IMG_DIR = "/home/zhaoyuhang/my_dataset_add_cup_cell/train/data_all"
TEST_LABEL_DIR = "/home/zhaoyuhang/my_dataset_add_cup_cell/train/labels_all"

# 创建文件夹
os.makedirs(COCO_LABEL_DIR, exist_ok=True)
os.makedirs(TEST_IMG_DIR, exist_ok=True)
os.makedirs(TEST_LABEL_DIR, exist_ok=True)

# ========= Step 1: COCO JSON 转 YOLO txt =========
with open(COCO_JSON) as f:
    coco = json.load(f)

images = coco["images"]
annotations = coco["annotations"]
categories = coco["categories"]

# 只保留目标类别并建立映射：cup=1, cell phone=2, remote=3
TARGET_CAT_NAMES = ["cup", "cell phone", "remote"]
cat_id_map = {}
cur_id = 1  # 从 1 开始，因为 0 留给 box

for cat in categories:
    if cat["name"] in TARGET_CAT_NAMES:
        cat_id_map[cat["id"]] = cur_id
        cur_id += 1

# 建立 image_id -> 图片信息映射
img_id_map = {img["id"]: img for img in images}

# 转换 COCO 标注
for ann in tqdm(annotations, desc="Converting COCO to YOLO"):
    img_info = img_id_map[ann["image_id"]]
    file_name = img_info["file_name"]

    # 只处理 COCO 图片文件夹里存在的图片
    if not os.path.exists(os.path.join(COCO_IMG_DIR, file_name)):
        continue

    # 只处理目标类别
    if ann["category_id"] not in cat_id_map:
        continue

    img_w = img_info["width"]
    img_h = img_info["height"]

    x, y, w, h = ann["bbox"]

    # 转 YOLO 格式
    x_center = (x + w/2) / img_w
    y_center = (y + h/2) / img_h
    w /= img_w
    h /= img_h

    class_id = cat_id_map[ann["category_id"]]

    txt_path = os.path.join(COCO_LABEL_DIR, file_name.replace(".jpg", ".txt"))
    with open(txt_path, "a") as f:
        f.write(f"{class_id} {x_center} {y_center} {w} {h}\n")

print("✅ COCO 转 YOLO 完成")

# ========= Step 2: 合并 box + COCO 到统一测试集 =========

# 复制 box 图片和标签
for img_file in os.listdir(BOX_IMG_DIR):
    if img_file.lower().endswith(('.jpg', '.png', '.jpeg')):
        shutil.copy(os.path.join(BOX_IMG_DIR, img_file),
                    os.path.join(TEST_IMG_DIR, img_file))
        # 对应 label
        label_file = img_file.replace(".jpg", ".txt")
        shutil.copy(os.path.join(BOX_LABEL_DIR, label_file),
                    os.path.join(TEST_LABEL_DIR, label_file))

# 复制 COCO 图片和生成的 YOLO label
for img_file in os.listdir(COCO_IMG_DIR):
    if img_file.lower().endswith(('.jpg', '.png', '.jpeg')):
        shutil.copy(os.path.join(COCO_IMG_DIR, img_file),
                    os.path.join(TEST_IMG_DIR, img_file))
        label_file = img_file.replace(".jpg", ".txt")
        src_label_path = os.path.join(COCO_LABEL_DIR, label_file)
        if os.path.exists(src_label_path):
            shutil.copy(src_label_path, os.path.join(TEST_LABEL_DIR, label_file))

print(f"✅ 测试集生成完成：{TEST_IMG_DIR} + {TEST_LABEL_DIR}")