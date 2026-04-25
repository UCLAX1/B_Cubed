import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import threading
import numpy as np

from .tensorrt_yolo import TRTModel, TRTVectorModel
# Load TensorRT engine instead of PyTorch model
model = TRTModel("/home/jetson-nano-x1/Documents/B_Cubed/models/hand_keypoints_trained.engine")

GESTURES = ["open_hand", "fist", "point", "thumbs_up"]

gesture_engine = TRTVectorModel(
    "/home/jetson-nano-x1/Documents/B_Cubed/models/gesture_classifier.engine",
    input_shape=(1, 42)  # only needed if the engine has dynamic dims
)

GESTURE_INPUT_MODE = "raw"  # Current deployed TensorRT gesture engine was trained on raw keypoints.
DEBUG_GESTURE_LOGITS = False



class RGBSubscriber(Node):
    def __init__(self):
        super().__init__('rgb_sub')
        self.bridge = CvBridge()
        self.last_frame = None
        self.lock = threading.Lock()

        self.sub = self.create_subscription(
            CompressedImage,
            '/zed/zed_node/rgb/color/rect/image/compressed',
            self.callback,
            1
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        with self.lock:
            self.last_frame = frame

    def timer_callback(self):
        with self.lock:
            if self.last_frame is None:
                return
            frame = self.last_frame.copy()

        annotated, detections = detect_hands(frame)
        cv2.imshow("YOLO TensorRT", annotated)
        cv2.waitKey(1)

# MediaPipe-style 21-point hand skeleton
HAND_EDGES = [
    (0,1),(1,2),(2,3),(3,4),
    (0,5),(5,6),(6,7),(7,8),
    (0,9),(9,10),(10,11),(11,12),
    (0,13),(13,14),(14,15),(15,16),
    (0,17),(17,18),(18,19),(19,20)
]

def iou_xyxy(a,b):
    b = np.asarray(b, dtype=np.float32)
    a = np.asarray(a, dtype=np.float32)

    x1 = np.maximum(a[0], b[:,0])
    y1 = np.maximum(a[1], b[:,1])
    x2 = np.maximum(a[2], b[:,2])
    y2 = np.maximum(a[3], b[:,3])

    intersection = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[:,2] - b[:,0]) * (b[:,3] - b[:,1])
    return intersection/ (area_a + area_b - intersection + 1e-9)

def nms_xyxy(boxes, scores, threshold = 0.45):
    if len(boxes) == 0:
        return []
    boxes = np.asarray(boxes, dtype=np.float32)
    scores = np.asarray(scores, dtype=np.float32)

    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        ious = iou_xyxy(boxes[i], boxes[rest])
        order = rest[ious < threshold]

    return keep

def prepare_gesture_input(kpts, bbox, mode=GESTURE_INPUT_MODE):
    pts_xy = np.asarray(kpts, dtype=np.float32)[:, :2].reshape(1, 42)

    if mode == "raw":
        return np.ascontiguousarray(pts_xy, dtype=np.float32)

    if mode == "bbox":
        x1, y1, x2, y2 = bbox
        bw = max(1.0, float(x2 - x1))
        bh = max(1.0, float(y2 - y1))
        cx = (x1 + x2) * 0.5
        cy = (y1 + y2) * 0.5
        pts_xy[:, 0::2] = (pts_xy[:, 0::2] - cx) / bw
        pts_xy[:, 1::2] = (pts_xy[:, 1::2] - cy) / bh
        return np.ascontiguousarray(pts_xy, dtype=np.float32)

    raise ValueError(f"Unknown gesture input mode: {mode}")

def _pose_layout_score(kpts, bbox):
    x1, y1, x2, y2 = bbox
    xy = kpts[:, :2]
    conf = kpts[:, 2]

    finite_score = float(np.isfinite(kpts).all())
    conf_score = float(np.mean((conf >= 0.0) & (conf <= 1.5)))
    inside_x = (xy[:, 0] >= x1 - 10.0) & (xy[:, 0] <= x2 + 10.0)
    inside_y = (xy[:, 1] >= y1 - 10.0) & (xy[:, 1] <= y2 + 10.0)
    inside_score = float(np.mean(inside_x & inside_y))

    limb_lengths = []
    for a, b in HAND_EDGES:
        limb_lengths.append(float(np.linalg.norm(xy[a] - xy[b])))
    limb_lengths = np.asarray(limb_lengths, dtype=np.float32)
    bbox_diag = max(1.0, float(np.hypot(x2 - x1, y2 - y1)))
    limb_score = float(np.mean(limb_lengths < bbox_diag * 0.8))

    return finite_score + conf_score + inside_score + limb_score

def _scale_pose_keypoints(kpts, sx, sy, wi, hi, wf, hf):
    kpts = np.asarray(kpts, dtype=np.float32).copy()
    if np.max(kpts[:, :2]) <= 1.5:
        kpts[:, 0] *= wi
        kpts[:, 1] *= hi

    kpts[:, 0] *= sx
    kpts[:, 1] *= sy
    kpts[:, 0] = np.clip(kpts[:, 0], 0.0, float(wf - 1))
    kpts[:, 1] = np.clip(kpts[:, 1], 0.0, float(hf - 1))
    return kpts

def parse_pose_keypoints(kpts_flat, bbox, nk, dim, sx, sy, wi, hi, wf, hf):
    kpts_flat = np.asarray(kpts_flat, dtype=np.float32)
    candidates = []

    interleaved = kpts_flat.reshape(nk, dim)[:, :3]
    candidates.append(_scale_pose_keypoints(interleaved, sx, sy, wi, hi, wf, hf))

    if dim == 3:
        grouped = np.column_stack((
            kpts_flat[:nk],
            kpts_flat[nk:2 * nk],
            kpts_flat[2 * nk:3 * nk],
        ))
        candidates.append(_scale_pose_keypoints(grouped, sx, sy, wi, hi, wf, hf))

    return max(candidates, key=lambda candidate: _pose_layout_score(candidate, bbox))

def detect_hands(frame, conf_thr=0.4, kpt_thr=0.25, nk=21, dim=3, hand_class=0):
    trt_output = model.infer(frame)

    Hf, Wf = frame.shape[:2]
    Hi, Wi = model.h, model.w

    detections = []
    if trt_output is None or trt_output.size == 0:
        return frame, detections

    # transpose heuristic (optional)
    if trt_output.ndim == 2 and trt_output.shape[0] in (68, 69, 70, 84, 85) and trt_output.shape[1] > trt_output.shape[0]:
        trt_output = trt_output.T

    sx = Wf / float(Wi)
    sy = Hf / float(Hi)

    row_len = trt_output.shape[1]
    kpt_len = nk * dim
    nc = row_len - 4 - kpt_len
    if nc <= 0:
        raise RuntimeError(f"Bad row_len={row_len}. Expected 4 + nc + {kpt_len}. Got nc={nc}.")

    kpt_start = 4 + nc

    boxes, scores, all_kpts = [], [], []

    for row in trt_output.astype(np.float32):
        cls_scores = row[4:4+nc]
        cls = int(np.argmax(cls_scores))
        conf = float(cls_scores[cls])

        if cls != hand_class or conf < conf_thr:
            continue

        cx, cy, w, h = row[0:4]
        x1 = (cx - w / 2.0) * sx
        y1 = (cy - h / 2.0) * sy
        x2 = (cx + w / 2.0) * sx
        y2 = (cy + h / 2.0) * sy

        x1 = max(0, min(Wf - 1, int(x1)))
        y1 = max(0, min(Hf - 1, int(y1)))
        x2 = max(0, min(Wf - 1, int(x2)))
        y2 = max(0, min(Hf - 1, int(y2)))
        if x2 <= x1 or y2 <= y1:
            continue

        kpts_flat = row[kpt_start:kpt_start + kpt_len]
        kpts = parse_pose_keypoints(kpts_flat, (x1, y1, x2, y2), nk, dim, sx, sy, Wi, Hi, Wf, Hf)

        boxes.append([x1, y1, x2, y2])
        scores.append(conf)
        all_kpts.append([tuple(point) for point in kpts])

    keep = nms_xyxy(boxes, scores)

    # Build final detections list AFTER NMS
    for i in keep:
        x1, y1, x2, y2 = boxes[i]
        conf = scores[i]
        kpts = all_kpts[i]
        detections.append({"bbox": (x1, y1, x2, y2), "conf": conf, "kpts": kpts})

        # draw bbox
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)

        # draw skeleton
        for a, b in HAND_EDGES:
            xa, ya, ca = kpts[a]
            xb, yb, cb = kpts[b]
            if ca < kpt_thr or cb < kpt_thr:
                continue
            cv2.line(frame, (int(xa), int(ya)), (int(xb), int(yb)), (255,0,0), 2)

        # draw keypoints
        for (xk, yk, ck) in kpts:
            if ck < kpt_thr:
                continue
            cv2.circle(frame, (int(xk), int(yk)), 3, (0,255,255), -1)

        cv2.putText(frame, f"hand {conf:.2f}", (x1, max(0, y1-10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        pts_xy = prepare_gesture_input(det["kpts"], det["bbox"])

        logits = gesture_engine.infer(pts_xy)  # likely (1,4)
        logits = np.asarray(logits).reshape(1, -1)
        pred = int(np.argmax(logits, axis=1)[0])
        gesture = GESTURES[pred]
        if DEBUG_GESTURE_LOGITS:
            print("gesture input range:", float(pts_xy.min()), float(pts_xy.max()),
                  "logits:", logits[0].tolist(), "pred:", gesture)

        cv2.putText(frame, gesture, (x1, max(0, y1 - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    return frame, detections



def main():
    rclpy.init()
    node = RGBSubscriber()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
