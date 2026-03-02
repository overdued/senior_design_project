import cv2
import numpy as np
import torch


CALIB_MODE = False

if CALIB_MODE:
    from det_utils import letterbox, scale_coords, nms
else:
    try:
        from det_utils import letterbox, scale_coords, nms
    except ImportError:
        from .det_utils import letterbox, scale_coords, nms


def preprocess_image(img_bgr, cfg):
    img, scale_ratio, pad_size = letterbox(img_bgr, new_shape=cfg["input_shape"])
    # bgr2rgb, HWC2CHW
    img = img[:, :, ::-1].transpose(2, 0, 1)
    img = np.ascontiguousarray(img, dtype=np.float32) / 255.0  # 确保是float32
    return img, scale_ratio, pad_size


def draw_bbox(bbox, img0, color, wt, names):
    det_result_str = ""
    if len(bbox) == 0:
        return img0
        
    for idx in range(bbox.shape[0]):
        if bbox.shape[1] < 6:  # 确保有足够的列
            continue
        class_id = int(bbox[idx][5])
        conf = float(bbox[idx][4])
        
        # 检查类别ID是否有效
        if class_id < 0 or class_id >= len(names):
            print(f"Warning: Invalid class ID {class_id}, skipping")
            continue
            
        # 使用传入的置信度阈值
        if conf < 0.05:
            continue
            
        x1, y1, x2, y2 = map(int, bbox[idx][:4])
        
        img0 = cv2.rectangle(
            img0,
            (x1, y1),
            (x2, y2),
            color,
            wt,
        )
        img0 = cv2.putText(
            img0,
            str(idx) + " " + names.get(class_id, f"unknown({class_id})"),
            (x1, y1 + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )
        img0 = cv2.putText(
            img0,
            "{:.4f}".format(conf),
            (x1, y1 + 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )
        det_result_str += "{} {} {} {} {} {}\n".format(
            names.get(class_id, f"unknown({class_id})"),
            str(conf),
            x1, y1, x2, y2,
        )
    return img0


def get_labels_from_txt(path):
    labels_dict = dict()
    with open(path) as f:
        for cat_id, label in enumerate(f.readlines()):
            labels_dict[cat_id] = label.strip()
    return labels_dict


def draw_prediction(pred, img_bgr, labels):
    img_dw = draw_bbox(pred, img_bgr, (0, 255, 0), 2, labels)
    return img_dw


def infer_image(img_bgr, model, class_names, cfg):
    img, scale_ratio, pad_size = preprocess_image(img_bgr, cfg)
    
    try:
        output = model.infer([img])[0]
        print(f"Raw model output shape: {output.shape}")
        
        # 确保输出是float32
        if output.dtype != np.float32:
            output = output.astype(np.float32)
            
    except Exception as e:
        print(f"Inference error: {e}")
        import traceback
        traceback.print_exc()
        raise

    # 转换为torch tensor进行NMS（强制使用float32）
    output_tensor = torch.from_numpy(output).float()
    
    boxout = nms(output_tensor, conf_thres=cfg["conf_thres"], iou_thres=cfg["iou_thres"])
    pred_all = boxout[0].cpu().numpy()
    
    # 添加类别ID过滤
    if len(pred_all) > 0 and pred_all.shape[1] >= 6:
        # 过滤无效类别ID
        valid_mask = (pred_all[:, 5] >= 0) & (pred_all[:, 5] < len(class_names))
        if np.any(valid_mask):
            pred_all = pred_all[valid_mask]
        else:
            pred_all = np.empty((0, 6))
            print("Warning: All detections have invalid class IDs")
    
    print(f"Final detections count: {len(pred_all)}")
    if len(pred_all) > 0:
        print(f"Class IDs range: [{int(pred_all[:, 5].min())}, {int(pred_all[:, 5].max())}]")
        print(f"Confidence range: [{pred_all[:, 4].min():.3f}, {pred_all[:, 4].max():.3f}]")
    
    # 坐标还原
    if len(pred_all) > 0:
        scale_coords(
            cfg["input_shape"],
            pred_all[:, :4],
            img_bgr.shape,
            ratio_pad=(scale_ratio, pad_size),
        )
    drawed_res = draw_prediction(pred_all, img_bgr, class_names)
    return pred_all, class_names, drawed_res


def xyxy2xywh(x):
    # Convert nx4 boxes from [x1, y1, x2, y2] to [x, y, w, h] where xy1=top-left, xy2=bottom-right
    y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
    # x center
    y[..., 0] = (x[..., 0] + x[..., 2]) / 2
    # y center
    y[..., 1] = (x[..., 1] + x[..., 3]) / 2
    # width
    y[..., 2] = x[..., 2] - x[..., 0]
    # height
    y[..., 3] = x[..., 3] - x[..., 1]
    return y