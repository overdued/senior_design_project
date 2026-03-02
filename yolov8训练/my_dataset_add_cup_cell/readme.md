在images_7中通过down_image.py文件下载box 1600张图片后，通过fix_labels_val.py进行标注后，生成该目录下的images和labels。
在通过prepare_data.py准备"cup", "cell phone", "remote"各800张，用labels_all.py来进行标注后，然后统一用didive.py来进行9：1的划分
开始训练。训练后模型数据在result_data中，符合预期，再用test_2.py进行测试，
使用了
    "yolov8x_pretrain": "/home/zhaoyuhang/yolov8x.pt",
    "yolo26x": "/home/zhaoyuhang/yolo26x.pt",
    "your_best_model": "/home/zhaoyuhang/runs/detect/train55/weights/best.pt",
    "yolo26n": "/home/zhaoyuhang/yolo26n.pt",
    "yolov8_image7": "/home/zhaoyuhang/yolov8x-oiv7.pt"五个模型，从大小和泛化性进行多个对比
生成model_class_comparison.png对比图和yolo_class_comparison.csv对比数据。
完成实验.