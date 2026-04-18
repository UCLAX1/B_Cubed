import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import threading
import numpy as np

from .tensorrt_yolo import TRTModel

# Load TensorRT engine instead of PyTorch model
model = TRTModel("/home/jetson-nano-x1/Documents/B_Cubed/models/yolo11n.engine")


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

        annotated, detections = detect_people(frame)
        cv2.imshow("YOLO TensorRT", annotated)
        cv2.waitKey(1)

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

def detect_people(frame):
    trt_output = model.infer(frame)   # expect (N,84) or similar
    # print("TRT out shape:", trt_output.shape)
    # print("Row length:", trt_output.shape[1])
    # print("First row first 10:", trt_output[0][:10])

    Hf, Wf = frame.shape[:2]
    Hi, Wi = model.h, model.w  # model input size (640x640)

    # detections = []
    boxes =[]
    scores = []
    detections = []

    if trt_output is None or trt_output.size == 0:
        return frame, detections

    # If output accidentally came as (84, N), transpose it
    if trt_output.ndim == 2 and trt_output.shape[0] in (84, 85) and trt_output.shape[1] > trt_output.shape[0]:
        trt_output = trt_output.T

    # Debug once (optional)
    # print("TRT out shape:", trt_output.shape, "sample:", trt_output[0, :10])

    sx = Wf / float(Wi)
    sy = Hf / float(Hi)

    for row in trt_output:
        row = row.astype(np.float32)

        # class scores start at index 4 for a 84-length row (4 + 80 classes)
        cls_scores = row[4:]
        cls = int(np.argmax(cls_scores))
        conf = float(cls_scores[cls])
        # print(conf)

        # COCO person = 0
        if cls != 0:
            continue
        if conf < 0.4:
            continue

        cx, cy, w, h = row[0:4]

        x1 = (cx - w /2.0) * sx
        y1 = (cy - h/2.0) * sy
        x2 = (cx + w/2.0) * sx
        y2 = (cy + h/2.0) * sy

        x1 = max(0, min(Wf - 1, int(x1)))
        y1 = max(0, min(Hf - 1, int(y1)))
        x2 = max(0, min(Wf - 1, int(x2)))
        y2 = max(0, min(Hf - 1, int(y2)))

        if x2 <= x1 or y2 <= y1:
            continue

        boxes.append([x1,y1,x2,y2])
        scores.append(conf)

    keep = nms_xyxy(boxes,scores)
    boxes = [boxes[i] for i in keep]
    scores = [scores[i] for i in keep]





    detections.append({'bbox': boxes, 'conf': scores})
        #
        # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # cv2.putText(frame, f"person {conf:.2f}", (x1, max(0, y1 - 10)),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    for (x1, y1, x2, y2), conf in zip(boxes, scores):
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            frame,
            f"person {conf:.2f}",
            (x1, max(0, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

    return frame, detections


def main():
    rclpy.init()
    node = RGBSubscriber()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
