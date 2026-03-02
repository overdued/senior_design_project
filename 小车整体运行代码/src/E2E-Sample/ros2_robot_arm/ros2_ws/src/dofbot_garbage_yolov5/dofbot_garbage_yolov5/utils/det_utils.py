"""
Copyright 2022 Huawei Technologies Co., Ltd

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import numpy as np
import torch
import torchvision
import cv2

def letterbox(
    img,
    new_shape=(640, 640),
    color=(114, 114, 114),
    auto=False,
    scaleFill=False,
    scaleup=True,
):
    # current shape [height, width]
    shape = img.shape[:2]
    # Resize image to a 32-pixel-multiple rectangle
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    # only scale down, do not scale up (for better test mAP)
    if not scaleup:
        r = min(r, 1.0)

    ## Compute padding
    # width, height ratios
    ratio = r, r
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    # wh padding
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    # minimum rectangle
    if auto:
        # wh padding
        dw, dh = np.mod(dw, 64), np.mod(dh, 64)
    # stretch
    elif scaleFill:
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        # width, height ratios
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]

    # divide padding into 2 sides
    dw /= 2
    dh /= 2

    # resize
    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    # add border
    img = cv2.copyMakeBorder(
        img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )
    return img, ratio, (dw, dh)


def xyxy2xywh(x):
    # Convert nx4 boxes from [x1, y1, x2, y2] to [x, y, w, h] where xy1=top-left, xy2=bottom-right
    y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
    # x center
    y[:, 0] = (x[:, 0] + x[:, 2]) / 2
    # y center
    y[:, 1] = (x[:, 1] + x[:, 3]) / 2
    # width
    y[:, 2] = x[:, 2] - x[:, 0]
    # height
    y[:, 3] = x[:, 3] - x[:, 1]
    return y


def non_max_suppression(
    prediction,
    conf_thres=0.25,
    iou_thres=0.45,
    classes=None,
    agnostic=False,
    multi_label=False,
    labels=(),
    max_det=300,
    nm=0,
):
    """Non-Maximum Suppression (NMS) on inference results to reject overlapping detections

    Returns:
         list of detections, on (n,6) tensor per image [xyxy, conf, cls]
    """

    # 确保输入是torch.Tensor且为float32
    if not isinstance(prediction, torch.Tensor):
        prediction = torch.from_numpy(prediction)
    
    # 强制转换为float32，避免Half类型问题
    if prediction.dtype != torch.float32:
        prediction = prediction.float()
    
    device = prediction.device
    print(f"Input prediction shape: {prediction.shape}")
    
    # 处理输出格式 [1, 605, 8400] -> [1, 8400, 605]
    if len(prediction.shape) == 3 and prediction.shape[1] < prediction.shape[2]:
        prediction = prediction.transpose(1, 2)
        print(f"Transposed prediction shape: {prediction.shape}")
    
    # 处理单个图像
    if len(prediction.shape) == 3:
        pred_single = prediction[0]  # [N, 4+classes]
    else:
        pred_single = prediction
    
    print(f"Pred single shape: {pred_single.shape}")
    
    # 检查维度是否合理
    if pred_single.shape[1] != 605:  # 4 + 601
        print(f"Warning: Expected 605 dimensions, got {pred_single.shape[1]}")
        return [torch.zeros((0, 6), device=device, dtype=torch.float32)]
    
    # YOLOv8格式 [x,y,w,h] + 601 classes
    boxes = pred_single[:, :4]
    cls_scores = pred_single[:, 4:]
    
    # 对于YOLOv8，直接使用类别分数的最大值作为置信度
    scores = torch.max(cls_scores, dim=1)[0]  # [N]
    classes_ids = torch.max(cls_scores, dim=1)[1]  # [N]
    
    print(f"Boxes shape: {boxes.shape}")
    print(f"Scores shape: {scores.shape}")
    print(f"Classes shape: {classes_ids.shape}")

    # Filter by confidence
    keep = scores >= conf_thres
    print(f"Keep mask sum: {keep.sum()}")
    
    if keep.sum() == 0:
        return [torch.zeros((0, 6), device=device, dtype=torch.float32)]

    boxes_filtered = boxes[keep]
    scores_filtered = scores[keep]
    classes_filtered = classes_ids[keep].float()

    print(f"Filtered boxes shape: {boxes_filtered.shape}")
    
    # 检查boxes格式，如果是cxcywh则转换为xyxy
    if boxes_filtered.shape[0] > 0 and boxes_filtered.shape[1] >= 4:
        # 检查是否需要转换坐标格式 (修复布尔类型错误)
        try:
            # 先转换为float再比较
            width_check = (boxes_filtered[:, 2].float() > boxes_filtered[:, 0].float()).float()
            if torch.mean(width_check) < 0.8:
                # 可能是cxcywh格式
                boxes_xyxy = torch.zeros_like(boxes_filtered)
                boxes_xyxy[:, 0] = boxes_filtered[:, 0] - boxes_filtered[:, 2] / 2  # x1
                boxes_xyxy[:, 1] = boxes_filtered[:, 1] - boxes_filtered[:, 3] / 2  # y1
                boxes_xyxy[:, 2] = boxes_filtered[:, 0] + boxes_filtered[:, 2] / 2  # x2
                boxes_xyxy[:, 3] = boxes_filtered[:, 1] + boxes_filtered[:, 3] / 2  # y2
                boxes_filtered = boxes_xyxy
                print("Converted cxcywh to xyxy format")
        except Exception as e:
            print(f"Coordinate format check error: {e}")
            # 如果检查失败，假设已经是xyxy格式

    # Apply torchvision NMS
    try:
        if boxes_filtered.shape[0] > 0:
            indices = torchvision.ops.nms(boxes_filtered, scores_filtered, iou_thres)
        else:
            indices = torch.tensor([], dtype=torch.long, device=device)
    except Exception as e:
        print(f"NMS kernel error: {e}")
        # 如果NMS失败，返回所有检测结果
        indices = torch.arange(boxes_filtered.shape[0], device=device)

    # Limit detections
    if len(indices) > max_det:
        indices = indices[:max_det]

    # Construct output: [x1, y1, x2, y2, conf, class_id]
    if len(indices) > 0 and boxes_filtered.shape[0] > 0:
        output_boxes = boxes_filtered[indices]
        output_scores = scores_filtered[indices]
        output_classes = classes_filtered[indices]
        
        detections = torch.cat([
            output_boxes,
            output_scores.unsqueeze(1),
            output_classes.unsqueeze(1)
        ], dim=1)
    else:
        detections = torch.zeros((0, 6), device=device, dtype=torch.float32)

    print(f"Final detections shape: {detections.shape}")
    return [detections]


def xywh2xyxy(x):
    # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
    y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
    # top left x
    y[:, 0] = x[:, 0] - x[:, 2] / 2
    # top left y
    y[:, 1] = x[:, 1] - x[:, 3] / 2
    # bottom right x
    y[:, 2] = x[:, 0] + x[:, 2] / 2
    # bottom right y
    y[:, 3] = x[:, 1] + x[:, 3] / 2
    return y


def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    # Rescale coords (xyxy) from img1_shape to img0_shape
    # calculate from img0_shape
    if ratio_pad is None:
        # gain  = old / new
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
        # wh padding
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (
            img1_shape[0] - img0_shape[0] * gain
        ) / 2
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    # x padding
    coords[:, [0, 2]] -= pad[0]
    # y padding
    coords[:, [1, 3]] -= pad[1]
    coords[:, :4] /= gain
    clip_coords(coords, img0_shape)
    return coords


def clip_coords(boxes, shape):
    ### Clip bounding xyxy bounding boxes to image shape (height, width)
    # faster individually
    if isinstance(boxes, torch.Tensor):
        # x1
        boxes[:, 0].clamp_(0, shape[1])
        # y1
        boxes[:, 1].clamp_(0, shape[0])
        # x2
        boxes[:, 2].clamp_(0, shape[1])
        # y2
        boxes[:, 3].clamp_(0, shape[0])
    # np.array (faster grouped)
    else:
        # x1, x2
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, shape[1])
        # y1, y2
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, shape[0])


def nms(box_out, conf_thres=0.4, iou_thres=0.5):
    # 确保输入是float32类型
    if isinstance(box_out, np.ndarray):
        box_out = torch.from_numpy(box_out)
    if box_out.dtype != torch.float32:
        box_out = box_out.float()
        
    # 保持接口不变，但内部逻辑适配YOLOv8
    try:
        boxout = non_max_suppression(
            box_out, 
            conf_thres=conf_thres, 
            iou_thres=iou_thres, 
            multi_label=False
        )
    except Exception as e:
        print(f"NMS error: {e}")
        import traceback
        traceback.print_exc()
        # 返回空结果而不是抛出异常
        device = box_out.device if hasattr(box_out, 'device') else torch.device('cpu')
        boxout = [torch.zeros((0, 6), device=device, dtype=torch.float32)]
    return boxout