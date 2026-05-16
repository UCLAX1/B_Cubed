import json
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from .tensorrt_yolo import TRTModel


class PersonTrackingNode(Node):
    def __init__(self):
        super().__init__("person_tracking")
        self.declare_parameter(
            "image_topic",
            "/zed/zed_node/rgb/color/rect/image/compressed",
        )
        self.declare_parameter(
            "engine_path",
            "/home/jetson-nano-x1/Documents/B_Cubed/models/yolo11n.engine",
        )
        self.declare_parameter("show_window", False)
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter(
            "annotated_image_topic",
            "/person_tracking/annotated_image/compressed",
        )
        self.declare_parameter("detection_topic", "/person_tracking/detections")
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("timer_period_sec", 0.05)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.engine_path = str(self.get_parameter("engine_path").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.publish_annotated_image = bool(
            self.get_parameter("publish_annotated_image").value
        )
        self.annotated_image_topic = str(
            self.get_parameter("annotated_image_topic").value
        )
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.nms_threshold = float(self.get_parameter("nms_threshold").value)
        self.timer_period_sec = max(
            0.01,
            float(self.get_parameter("timer_period_sec").value),
        )

        self.last_frame = None
        self.last_header = None
        self.lock = threading.Lock()
        self.model = TRTModel(self.engine_path)

        self.detection_pub = self.create_publisher(
            String,
            self.detection_topic,
            10,
        )
        self.annotated_pub = None
        if self.publish_annotated_image:
            self.annotated_pub = self.create_publisher(
                CompressedImage,
                self.annotated_image_topic,
                10,
            )

        self.sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.callback,
            1,
        )

        self.timer = self.create_timer(self.timer_period_sec, self.timer_callback)
        self.get_logger().info(
            "Person tracking ready: "
            f"image_topic={self.image_topic}, engine_path={self.engine_path}, "
            f"detection_topic={self.detection_topic}."
        )

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Unable to decode compressed image frame.")
            return

        with self.lock:
            self.last_frame = frame
            self.last_header = msg.header

    def timer_callback(self):
        with self.lock:
            if self.last_frame is None:
                return
            frame = self.last_frame.copy()
            header = self.last_header

        annotated, detections = detect_people(
            frame,
            self.model,
            confidence_threshold=self.confidence_threshold,
            nms_threshold=self.nms_threshold,
        )
        self._publish_detections(detections)

        if self.annotated_pub is not None:
            self._publish_annotated_image(annotated, header)

        if self.show_window:
            cv2.imshow("Person Tracking", annotated)
            cv2.waitKey(1)

    def _publish_detections(self, detections):
        payload = {"detections": detections}
        self.detection_pub.publish(String(data=json.dumps(payload)))

    def _publish_annotated_image(self, frame, header):
        ok, encoded = cv2.imencode(".jpg", frame)
        if not ok:
            self.get_logger().warning("Unable to encode annotated image.")
            return
        msg = CompressedImage()
        if header is not None:
            msg.header = header
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.annotated_pub.publish(msg)


def iou_xyxy(a, b):
    b = np.asarray(b, dtype=np.float32)
    a = np.asarray(a, dtype=np.float32)

    x1 = np.maximum(a[0], b[:, 0])
    y1 = np.maximum(a[1], b[:, 1])
    x2 = np.maximum(a[2], b[:, 2])
    y2 = np.maximum(a[3], b[:, 3])

    intersection = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    return intersection / (area_a + area_b - intersection + 1e-9)


def nms_xyxy(boxes, scores, threshold=0.45):
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


def detect_people(frame, model, confidence_threshold=0.4, nms_threshold=0.45):
    trt_output = model.infer(frame)   # expect (N,84) or similar
    # print("TRT out shape:", trt_output.shape)
    # print("Row length:", trt_output.shape[1])
    # print("First row first 10:", trt_output[0][:10])

    Hf, Wf = frame.shape[:2]
    Hi, Wi = model.h, model.w  # model input size (640x640)

    # detections = []
    boxes = []
    scores = []
    detections = []

    if trt_output is None or trt_output.size == 0:
        return frame, detections

    # If output accidentally came as (84, N), transpose it
    if (
        trt_output.ndim == 2
        and trt_output.shape[0] in (84, 85)
        and trt_output.shape[1] > trt_output.shape[0]
    ):
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
        if conf < confidence_threshold:
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

        boxes.append([x1, y1, x2, y2])
        scores.append(conf)

    keep = nms_xyxy(boxes, scores, threshold=nms_threshold)
    boxes = [boxes[i] for i in keep]
    scores = [scores[i] for i in keep]

    for (x1, y1, x2, y2), conf in zip(boxes, scores):
        detections.append(
            {
                "bbox": [int(x1), int(y1), int(x2), int(y2)],
                "confidence": float(conf),
            }
        )
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
    node = PersonTrackingNode()
    try:
        rclpy.spin(node)
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
