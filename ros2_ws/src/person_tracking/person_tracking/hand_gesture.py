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

GESTURE_INPUT_MODE = "raw"  # The current classifier was trained on raw keypoint pixels.
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

        kpts_raw = row[kpt_start:kpt_start + kpt_len].reshape(nk, dim)  # (21,3)

        # If kpts look normalized, scale to model input pixels
        if np.max(kpts_raw[:, :2]) <= 1.5:
            kpts_raw[:, 0] *= Wi
            kpts_raw[:, 1] *= Hi

        # Scale to original frame
        kpts = []
        for i in range(nk):
            xk = float(kpts_raw[i, 0] * sx)
            yk = float(kpts_raw[i, 1] * sy)
            ck = float(kpts_raw[i, 2]) if dim >= 3 else 1.0
            xk = max(0.0, min(float(Wf - 1), xk))
            yk = max(0.0, min(float(Hf - 1), yk))
            kpts.append((xk, yk, ck))

        boxes.append([x1, y1, x2, y2])
        scores.append(conf)
        all_kpts.append(kpts)

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
