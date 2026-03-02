# --- YOLOv8 OpenImagesV7（601类）Jupyter最简推理显示 ---
# 依赖: onnxruntime, opencv-python, numpy, pillow, IPython

import os, time, io
from typing import List, Tuple
import numpy as np
import cv2
from PIL import Image
from IPython.display import display, clear_output, Image as IPyImage
from ais_bench.infer.interface import InferSession

NUM_CLASSES = 80   #只有80个类？
INPUT_SIZE = 640
CONF_THR = 0.25
IOU_THR = 0.5

def load_names(path: str) -> List[str]:
    with open(path, 'r', encoding='utf-8') as f:
        names = [ln.strip() for ln in f if ln.strip()]
    assert len(names) == NUM_CLASSES, f"classes.txt 应为 {NUM_CLASSES} 行，实际 {len(names)}"
    return names

def letterbox(im: np.ndarray, new_size=640, color=(114,114,114)):
    h, w = im.shape[:2]
    scale = min(new_size / h, new_size / w)
    nh, nw = int(round(h * scale)), int(round(w * scale))
    im_resized = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top = (new_size - nh) // 2
    left = (new_size - nw) // 2
    canvas = np.full((new_size, new_size, 3), color, dtype=np.uint8)
    canvas[top:top+nh, left:left+nw] = im_resized
    return canvas, scale, left, top, (h, w)

def nms(boxes: np.ndarray, scores: np.ndarray, iou_thr=0.5):
    if len(boxes) == 0: return []
    x1, y1, x2, y2 = boxes.T
    areas = (x2-x1) * (y2-y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2-xx1)
        h = np.maximum(0.0, yy2-yy1)
        inter = w*h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        inds = np.where(iou <= iou_thr)[0]
        order = order[inds + 1]
    return keep

class YOLO_OI601:
    def __init__(self, om_path: str, classes_path: str, conf_thr=CONF_THR, iou_thr=IOU_THR):
        """
            om_path: 模型文件路径
            classes_path: 类别文件路径
            conf_thr: 置信度阈值
            iou_thr: NMS IoU 阈值
        """
        self.names = load_names(classes_path)
        self.session = InferSession(0, om_path)
        self.conf_thr = conf_thr
        self.iou_thr = iou_thr

    def infer(self, bgr: np.ndarray):
        H, W = bgr.shape[:2]
        lb_img, scale, pad_x, pad_y, (h0, w0) = letterbox(bgr, INPUT_SIZE)
        blob = lb_img[:, :, ::-1].astype(np.float32) / 255.0
        blob = np.transpose(blob, (2,0,1))[None]  # [1,3,640,640]

        t0 = time.time()
        out = self.session.infer([blob])[0]  # [1, N, 4+601] 或 [1, 4+601, N]

        infer_ms = (time.time() - t0) * 1000.0

        pred = np.squeeze(out, 0)
        if pred.shape[0] < pred.shape[1]:
            pred = pred.T  # [N, 4+601]
        assert pred.shape[1] in (4+NUM_CLASSES, 5+NUM_CLASSES), f"输出维度不符: {pred.shape}"

        # 无 obj_conf（4+601）；若是 5+601 请看文末补丁
        if pred.shape[1] == 4 + NUM_CLASSES:
            boxes = pred[:, :4]
            cls_all = pred[:, 4:]
            scores = cls_all.max(axis=1)
            cls_ids = cls_all.argmax(axis=1)
        else:
            # 5+601（带 obj_conf）的兼容（若你的模型如此，可直接保留）
            boxes = pred[:, :4]
            obj = pred[:, 4:5]
            cls_all = pred[:, 5:]
            scores_all = obj * cls_all
            scores = scores_all.max(axis=1)
            cls_ids = scores_all.argmax(axis=1)

        mask = scores >= self.conf_thr
        if not np.any(mask):
            return [], infer_ms
        boxes, scores, cls_ids = boxes[mask], scores[mask], cls_ids[mask]

        # 处理坐标：支持 xyxy 或 cxcywh；也兼容 0..1 归一化
        b = boxes.copy()
        if np.percentile(np.abs(b), 98) <= 1.5:  # 归一化坐标
            b[:, [0,2]] *= INPUT_SIZE
            b[:, [1,3]] *= INPUT_SIZE
        xyxy_like = np.mean((b[:,2] > b[:,0]) & (b[:,3] > b[:,1])) > 0.7
        if not xyxy_like:  # cxcywh -> xyxy
            cx, cy, bw, bh = b[:,0], b[:,1], b[:,2], b[:,3]
            x1, y1, x2, y2 = cx - bw/2, cy - bh/2, cx + bw/2, cy + bh/2
            b = np.stack([x1,y1,x2,y2], axis=1)

        # 去 pad/缩放 -> 原图
        b[:, [0,2]] = (b[:, [0,2]] - pad_x) / scale
        b[:, [1,3]] = (b[:, [1,3]] - pad_y) / scale
        b[:, [0,2]] = np.clip(b[:, [0,2]], 0, W)
        b[:, [1,3]] = np.clip(b[:, [1,3]], 0, H)

        keep = nms(b, scores, self.iou_thr)
        #dets = [(b[i].astype(int), float(scores[i]), int(cls_ids[i])) for i in keep]
        #这里进行了修改
        dets = []
        for i in keep:
            x1, y1, x2, y2 = b[i].astype(int).tolist()
            conf = float(scores[i])
            cls = int(cls_ids[i])
            dets.append([x1, y1, x2, y2, conf, cls])
        return dets, infer_ms

    def draw(self, img: np.ndarray, dets):
        vis = img.copy()
        H, W = vis.shape[:2]
        for (x1,y1,x2,y2), sc, cid in dets:
            color = (0,255,0)
            cv2.rectangle(vis, (x1,y1), (x2,y2), color, 2)

            label = f"{self.names[cid]} {sc:.2f}"
            (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            pad = 2
            bar_h = th + baseline + 4  # 文本条高度

            # 默认放在框外上方
            tx = x1
            ty = y1 - 4
            box_y1 = y1 - bar_h
            box_y2 = y1

            # 如果顶边越界，改为放在框内上方
            if box_y1 < 0:
                box_y1 = y1
                box_y2 = min(y1 + bar_h, H)
                ty = box_y2 - baseline - 2

            # 如果右边越界，左移
            if tx + tw + 2*pad > W:
                tx = max(0, W - tw - 2*pad)

            # 画背景条与文字
            box_x1 = max(0, tx - pad)
            box_x2 = min(W, tx + tw + pad)
            cv2.rectangle(vis, (box_x1, box_y1), (box_x2, box_y2), color, -1)
            cv2.putText(vis, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0,0,0), 1, cv2.LINE_AA)
        return vis

def show_frame_jupyter(bgr: np.ndarray, max_w=960):
    h, w = bgr.shape[:2]
    if w > max_w:
        scale = max_w / w
        bgr = cv2.resize(bgr, (int(w*scale), int(h*scale)))
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(rgb)
    buf = io.BytesIO(); pil.save(buf, format='JPEG', quality=85)
    display(IPyImage(data=buf.getvalue()))

# import cv2, time, threading, queue

# # ====== 配置 ======
# model_path = "/root/Yolotest/yolov8m.om"
# classes_path = "/root/Yolotest/classes.txt"
# source = 0                  # 0/1 或 "video.mp4"
# display_width = None        # None = 不缩放，避免额外开销
# infer_interval = 0.0        # 0 = 每帧推理
# use_stride = False          # 不步进
# show_hud = False            # 如需看数字可 True，但会有轻微开销
# draw_boxes = True           # 如果只想看极限 FPS，可设 False

# # ====== 初始化 ======
# det = YOLO_OI601(model_path, classes_path)

# cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
# if not cap.isOpened():
#     raise RuntimeError(f"无法打开视频源: {source}")

# # 最大化相机吞吐（按需生效）
# try:
#     cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
# except Exception:
#     pass
# # 尝试 MJPG（很多 UVC 摄像头更快）
# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

# win = "YOLO-OI601"
# cv2.namedWindow(win, cv2.WINDOW_NORMAL)

# # ====== 后台读帧线程：始终保留最新一帧，丢弃旧帧 ======
# frame_lock = threading.Lock()
# latest = {"frame": None, "ok": False, "ts": 0.0, "eof": False}

# def grabber():
#     while True:
#         ok, f = cap.read()
#         if not ok:
#             # 文件源读到结尾则循环
#             if isinstance(source, str):
#                 cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
#                 continue
#             latest["eof"] = True
#             time.sleep(0.001)
#             continue
#         with frame_lock:
#             latest["frame"] = f
#             latest["ok"] = True
#             latest["ts"] = time.time()

# t = threading.Thread(target=grabber, daemon=True)
# t.start()

# # ====== 主循环：拉满 ======
# last_infer_s = 0.0
# src_frames = 0
# src_t0 = time.time()
# disp_frames = 0
# disp_t0 = time.time()

# def overlay_hud(img, fps, infer_s):
#     x, y = 10, 26
#     green = (0, 255, 0)
#     cv2.putText(img, f"FPS: {fps:.1f}", (x, y),
#                 cv2.FONT_HERSHEY_SIMPLEX, 0.8, green, 2, cv2.LINE_AA)
#     cv2.putText(img, f"Infer: {infer_s*1000:.0f} ms", (x, y+28),
#                 cv2.FONT_HERSHEY_SIMPLEX, 0.8, green, 2, cv2.LINE_AA)

# try:
#     while True:
#         # 获取最新帧（无则忙等极短时间，避免大 sleep）
#         f = None
#         with frame_lock:
#             if latest["ok"]:
#                 f = latest["frame"]
#         if f is None:
#             time.sleep(0.0005)  # 极短让步，避免 100% CPU
#             continue

#         src_frames += 1
#         srcFPS = src_frames / max(time.time() - src_t0, 1e-6)

#         # 推理（每帧）
#         t0 = time.time()
#         try:
#             dets, infer_ms = det.infer(f)
#             last_infer_s = infer_ms / 1000.0
#         except Exception:
#             last_infer_s = 0.0
#             dets = None

#         # 直接在原帧上绘制，避免 copy
#         vis = f
#         if draw_boxes and dets is not None:
#             vis = det.draw(vis, dets)  # 确保内部不再 copy；如会 copy，可改为 in-place 版本

#         if display_width is not None and vis.shape[1] != display_width:
#             h, w = vis.shape[:2]
#             vis = cv2.resize(vis, (display_width, int(h * display_width / w)), interpolation=cv2.INTER_LINEAR)

#         if show_hud:
#             overlay_hud(vis, fps=srcFPS, infer_s=last_infer_s)

#         cv2.imshow(win, vis)
#         disp_frames += 1

#         # 打印一次/秒
#         if time.time() - disp_t0 >= 1.0:
#             dispFPS = disp_frames / (time.time() - disp_t0)
#             print(f"srcFPS: {srcFPS:.1f} | dispFPS: {dispFPS:.1f} | infer: {last_infer_s*1000:.0f} ms")
#             disp_frames, disp_t0 = 0, time.time()
#             src_frames, src_t0 = 0, time.time()

#         # 不阻塞或最小阻塞
#         if (cv2.waitKey(1) & 0xFF) == ord('q'):
#             break

# except KeyboardInterrupt:
#     pass
# finally:
#     cap.release()
#     cv2.destroyAllWindows()