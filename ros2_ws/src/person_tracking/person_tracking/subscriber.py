import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import threading
import numpy as np

from tensorrt_yolo import TRTModel

# Load TensorRT engine instead of PyTorch model
model = TRTModel("/home/jetson-nano-x1/Downloads/B_Cubed/models/yolo11n.engine")


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
            10
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


def detect_people(frame):
    trt_output = model.infer(frame)

    detections = []
    for row in trt_output:
        conf = row[4]
        if conf < 0.4:
            continue

        cls = int(np.argmax(row[5:]))

        # 0 = person for COCO models
        if cls != 0:
            continue

        # xywh → xyxy conversion
        cx, cy, w, h = row[0:4]
        x1 = int(cx - w / 2)
        y1 = int(cy - h / 2)
        x2 = int(cx + w / 2)
        y2 = int(cy + h / 2)

        detections.append({'bbox': (x1, y1, x2, y2), 'conf': float(conf)})

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{conf:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return frame, detections


def main():
    rclpy.init()
    node = RGBSubscriber()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
