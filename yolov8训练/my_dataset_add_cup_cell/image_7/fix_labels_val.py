# -*- coding: utf-8 -*-
import os
import pandas as pd
from tqdm import tqdm

# ===================== 所有路径都在这里，明明白白 =====================

# 【类别映射表】：我们就用 train 目录下的这一个，足够了
CLASSES_CSV_PATH = "/home/zhaoyuhang/fiftyone/open-images-v7/validation/metadata/classes.csv"

# 【训练集】
TRAIN_LABEL_CSV = "/home/zhaoyuhang/fiftyone/open-images-v7/train/labels/detections.csv"
TRAIN_IMAGE_FOLDER = "/home/zhaoyuhang/my_dataset/images/train"
TRAIN_LABEL_OUTPUT_FOLDER = "/home/zhaoyuhang/my_dataset/labels/train"

# 【验证集】
VAL_LABEL_CSV = "/home/zhaoyuhang/fiftyone/open-images-v7/validation/labels/detections.csv"
VAL_IMAGE_FOLDER = "/home/zhaoyuhang/my_dataset/images/val"
VAL_LABEL_OUTPUT_FOLDER = "/home/zhaoyuhang/my_dataset/labels/val"

# 【要检测的类别】
CLASSES_TO_DETECT = ["Box", "Package", "Carton"]

# =======================================================================

def process(csv_path, img_folder, out_folder, name2id, id2name, valid_names, stage_name):
    print(f"\n>>> 正在处理 {stage_name} ...")
    
    # 1. 检查文件
    if not os.path.exists(csv_path):
        print(f"❌ 找不到 CSV: {csv_path}")
        return
    if not os.path.exists(img_folder):
        print(f"❌ 找不到图片文件夹: {img_folder}")
        return

    # 2. 读取并筛选标注
    df = pd.read_csv(csv_path)
    target_ids = [name2id[name] for name in valid_names]
    df = df[df['LabelName'].isin(target_ids)]
    
    # 3. 准备输出
    os.makedirs(out_folder, exist_ok=True)
    img_files = [f for f in os.listdir(img_folder) if f.lower().endswith(('.jpg', '.png'))]
    annot_map = dict(list(df.groupby('ImageID')))
    
    # 4. 生成 txt
    count = 0
    for img_file in tqdm(img_files, desc=stage_name):
        img_id = os.path.splitext(img_file)[0]
        if img_id in annot_map:
            with open(os.path.join(out_folder, f"{img_id}.txt"), 'w') as f:
                for _, row in annot_map[img_id].iterrows():
                    cls_idx = valid_names.index(id2name[row['LabelName']])
                    xc = (row['XMin'] + row['XMax']) / 2.0
                    yc = (row['YMin'] + row['YMax']) / 2.0
                    w = row['XMax'] - row['XMin']
                    h = row['YMax'] - row['YMin']
                    f.write(f"{cls_idx} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}\n")
            count += 1
    print(f"✅ {stage_name} 完成，生成了 {count} 个标签")

if __name__ == "__main__":
    print("🚀 启动...")
    
    # 加载类别
    cls_df = pd.read_csv(CLASSES_CSV_PATH, names=['id', 'name'], header=None)
    id2name = dict(zip(cls_df['id'], cls_df['name']))
    name2id = dict(zip(cls_df['name'], cls_df['id']))
    
    # 检查类别是否存在
    valid = [n for n in CLASSES_TO_DETECT if n in name2id]
    print(f"有效类别列表: {valid}")
    
    # 跑起来
    process(TRAIN_LABEL_CSV, TRAIN_IMAGE_FOLDER, TRAIN_LABEL_OUTPUT_FOLDER, name2id, id2name, valid, "训练集(Train)")
    process(VAL_LABEL_CSV, VAL_IMAGE_FOLDER, VAL_LABEL_OUTPUT_FOLDER, name2id, id2name, valid, "验证集(Val)")
    
    print("\n🎉 全部搞定！检查一下 labels 文件夹里有没有 txt，有就可以开始训练了！")