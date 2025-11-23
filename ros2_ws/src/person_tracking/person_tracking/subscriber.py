import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import threading
import numpy as np

model = YOLO("../models/yolo11n.pt")


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
        # frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        with self.lock:
           self.last_frame = frame
    
    def timer_callback(self):
        # Only run YOLO if we have a frame
        with self.lock:
            if self.last_frame is None:
                return
            frame = self.last_frame.copy()

        annotated, detections = detect_people(frame)
        cv2.imshow("YOLO", annotated)
        cv2.waitKey(1)

def detect_people(frame):
    """
    Run YOLOv11 person detection on an OpenCV BGR frame.

    Returns:
        annotated_frame: the frame with bounding boxes drawn
        detections: list of dicts with {x1, y1, x2, y2, conf}
    """
    results = model(frame, device = 0, verbose=False)
    detections = []

    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls = int(box.cls[0])
            label = model.names[cls]

            # Only keep "person"
            if label != "person":
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])

            detections.append({
                'bbox': (x1, y1, x2, y2),
                'conf': conf
            })

            # Draw the bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return frame, detections

def main():
    rclpy.init()
    node = RGBSubscriber()
    rclpy.spin(node)
    print("node started")

if __name__ == '__main__':
    main()