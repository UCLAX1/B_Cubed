import json
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    import mediapipe as mp
except ImportError as exc:
    mp = None
    _MEDIAPIPE_IMPORT_ERROR = exc


PACKAGE_NAME = "gesture_recognition"
PRETRAINED_GESTURE_RECOGNIZER_URL = (
    "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/"
    "gesture_recognizer/float16/1/gesture_recognizer.task"
)

HAND_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
]


def require_mediapipe():
    if mp is None:
        raise RuntimeError(
            "MediaPipe is not installed. Install the standalone requirements with "
            "`python3 -m pip install mediapipe`, then rebuild/source the ROS workspace."
        ) from _MEDIAPIPE_IMPORT_ERROR
    return mp


def create_recognizer(model_path: str, num_hands: int):
    mediapipe = require_mediapipe()
    options = mediapipe.tasks.vision.GestureRecognizerOptions(
        base_options=mediapipe.tasks.BaseOptions(model_asset_path=model_path),
        running_mode=mediapipe.tasks.vision.RunningMode.VIDEO,
        num_hands=num_hands,
    )
    return mediapipe.tasks.vision.GestureRecognizer.create_from_options(options)


def first_gesture_name(result) -> tuple[str, float]:
    if not result.gestures or not result.gestures[0]:
        return "None", 0.0
    category = result.gestures[0][0]
    return category.category_name, float(category.score)


def draw_first_hand(frame_bgr: np.ndarray, result) -> None:
    if not result.hand_landmarks:
        return

    h, w = frame_bgr.shape[:2]
    landmarks = result.hand_landmarks[0]
    points = np.array([[lm.x, lm.y, lm.z] for lm in landmarks], dtype=np.float32)
    pixels = np.column_stack((points[:, 0] * w, points[:, 1] * h)).astype(int)

    for a, b in HAND_EDGES:
        cv2.line(frame_bgr, tuple(pixels[a]), tuple(pixels[b]), (0, 255, 0), 2)
    for x, y in pixels:
        cv2.circle(frame_bgr, (int(x), int(y)), 3, (0, 255, 255), -1)


class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__("gesture_recognition")
        self._declare_parameters()

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.image_is_compressed = bool(self.get_parameter("image_is_compressed").value)
        self.model_path = self._resolve_model_path(str(self.get_parameter("model_path").value))
        self.num_hands = int(self.get_parameter("num_hands").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.publish_annotated_image = bool(self.get_parameter("publish_annotated_image").value)
        self.gesture_topic = str(self.get_parameter("gesture_topic").value)
        self.annotated_image_topic = str(self.get_parameter("annotated_image_topic").value)

        self.bridge = CvBridge() if CvBridge is not None else None
        self.warned_image_encodings: set[str] = set()
        self.start_time = time.monotonic()
        self.last_timestamp_ms = -1
        self.last_logged_gesture: str | None = None

        self.recognizer = create_recognizer(self.model_path, self.num_hands)
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        msg_type = CompressedImage if self.image_is_compressed else Image
        callback = self._compressed_image_callback if self.image_is_compressed else self._image_callback
        self.image_sub = self.create_subscription(msg_type, self.image_topic, callback, image_qos)
        self.gesture_pub = self.create_publisher(String, self.gesture_topic, 10)

        self.annotated_pub = None
        if self.publish_annotated_image:
            self.annotated_pub = self.create_publisher(
                CompressedImage,
                self.annotated_image_topic,
                10,
            )

        self.get_logger().info(
            "Gesture recognition subscribed to "
            f"{self.image_topic} (compressed={self.image_is_compressed}); "
            f"model={self.model_path}; result_topic={self.gesture_topic}."
        )
        if self.annotated_pub is not None:
            self.get_logger().info(
                f"Publishing annotated frames on {self.annotated_image_topic}."
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image/compressed")
        self.declare_parameter("image_is_compressed", True)
        self.declare_parameter("model_path", "")
        self.declare_parameter("download_pretrained_if_missing", False)
        self.declare_parameter("num_hands", 1)
        self.declare_parameter("show_window", False)
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("gesture_topic", "/gesture_recognition/result")
        self.declare_parameter(
            "annotated_image_topic",
            "/gesture_recognition/annotated_image/compressed",
        )

    def _resolve_model_path(self, configured_model_path: str) -> str:
        configured_model_path = configured_model_path.strip()
        if configured_model_path:
            model_path = Path(configured_model_path).expanduser()
            if model_path.is_file():
                return str(model_path)
            raise RuntimeError(f"Configured MediaPipe gesture model does not exist: {model_path}")

        for candidate in self._model_candidates():
            if candidate.is_file():
                return str(candidate)

        if bool(self.get_parameter("download_pretrained_if_missing").value):
            return self._download_pretrained_model()

        candidates = "\n".join(f"  - {candidate}" for candidate in self._model_candidates())
        raise RuntimeError(
            "Could not find a MediaPipe gesture recognizer task model. "
            "Set the `model_path` parameter, add gesture_recognizer.task to this package, "
            "or enable `download_pretrained_if_missing`.\n"
            f"Checked:\n{candidates}"
        )

    def _model_candidates(self) -> list[Path]:
        candidates: list[Path] = []

        try:
            share_dir = Path(get_package_share_directory(PACKAGE_NAME))
            candidates.extend(
                [
                    share_dir / "models" / "gesture_recognizer.task",
                    share_dir / "models" / "gesture_recognizer_pretrained.task",
                ]
            )
        except PackageNotFoundError:
            pass

        for parent in Path(__file__).resolve().parents:
            candidates.extend(
                [
                    parent / "models" / "gesture_recognizer.task",
                    parent / "models" / "gesture_recognizer_pretrained.task",
                    parent / "gesture_classification" / "exported_model" / "gesture_recognizer.task",
                    parent / "gesture_classification" / "gesture_recognizer_pretrained.task",
                ]
            )

        unique_candidates: list[Path] = []
        seen: set[Path] = set()
        for candidate in candidates:
            if candidate not in seen:
                unique_candidates.append(candidate)
                seen.add(candidate)
        return unique_candidates

    def _download_pretrained_model(self) -> str:
        target = self._default_download_target()
        target.parent.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Downloading pretrained MediaPipe gesture model to {target}.")
        try:
            urllib.request.urlretrieve(PRETRAINED_GESTURE_RECOGNIZER_URL, target)
        except (OSError, urllib.error.URLError) as exc:
            raise RuntimeError(
                "Could not download the pretrained MediaPipe Gesture Recognizer. "
                f"Download it manually from {PRETRAINED_GESTURE_RECOGNIZER_URL} "
                f"and save it as {target}."
            ) from exc
        return str(target)

    def _default_download_target(self) -> Path:
        try:
            share_dir = Path(get_package_share_directory(PACKAGE_NAME))
            return share_dir / "models" / "gesture_recognizer_pretrained.task"
        except PackageNotFoundError:
            return Path(__file__).resolve().parents[1] / "models" / "gesture_recognizer_pretrained.task"

    def _compressed_image_callback(self, msg: CompressedImage) -> None:
        buffer = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Failed to decode compressed image frame.")
            return
        self._process_frame(frame, msg.header)

    def _image_callback(self, msg: Image) -> None:
        frame = self._image_msg_to_bgr(msg)
        if frame is None:
            return
        self._process_frame(frame, msg.header)

    def _image_msg_to_bgr(self, msg: Image) -> np.ndarray | None:
        if self.bridge is not None:
            return np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))

        encoding = (msg.encoding or "").lower()
        if encoding not in {"bgr8", "rgb8", "bgra8", "rgba8", "mono8"}:
            if encoding not in self.warned_image_encodings:
                self.get_logger().warning(
                    f"Unsupported raw image encoding '{msg.encoding}'. "
                    "Install cv_bridge or switch image_is_compressed back to true."
                )
                self.warned_image_encodings.add(encoding)
            return None

        row = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.step))

        if encoding in {"bgr8", "rgb8"}:
            channels = 3
        elif encoding in {"bgra8", "rgba8"}:
            channels = 4
        else:
            channels = 1

        frame = row[:, : msg.width * channels].reshape((msg.height, msg.width, channels))
        if encoding == "bgr8":
            return frame.copy()
        if encoding == "rgb8":
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        return cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2BGR)

    def _process_frame(self, frame_bgr: np.ndarray, header) -> None:
        frame_rgb = np.ascontiguousarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        timestamp_ms = self._timestamp_ms(header)

        result = self.recognizer.recognize_for_video(mp_image, timestamp_ms)
        gesture, confidence = first_gesture_name(result)
        self._publish_result(header, gesture, confidence)

        if self.annotated_pub is not None or self.show_window:
            draw_first_hand(frame_bgr, result)
            cv2.putText(
                frame_bgr,
                f"{gesture} {confidence:.2f}",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
            )

        if self.annotated_pub is not None:
            self._publish_annotated_frame(frame_bgr, header)

        if self.show_window:
            cv2.imshow("MediaPipe Gesture Recognizer", frame_bgr)
            cv2.waitKey(1)

    def _timestamp_ms(self, header) -> int:
        stamp = getattr(header, "stamp", None)
        if stamp is not None and (stamp.sec or stamp.nanosec):
            timestamp_ms = int(stamp.sec * 1000 + stamp.nanosec // 1_000_000)
        else:
            timestamp_ms = int((time.monotonic() - self.start_time) * 1000)

        if timestamp_ms <= self.last_timestamp_ms:
            timestamp_ms = self.last_timestamp_ms + 1
        self.last_timestamp_ms = timestamp_ms
        return timestamp_ms

    def _publish_result(self, header, gesture: str, confidence: float) -> None:
        payload = {
            "gesture": gesture,
            "confidence": round(confidence, 4),
            "stamp": {
                "sec": int(header.stamp.sec),
                "nanosec": int(header.stamp.nanosec),
            },
        }
        self.gesture_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

        if gesture != self.last_logged_gesture:
            self.get_logger().info(f"Gesture: {gesture} ({confidence:.2f})")
            self.last_logged_gesture = gesture

    def _publish_annotated_frame(self, frame_bgr: np.ndarray, header) -> None:
        ok, encoded = cv2.imencode(".jpg", frame_bgr)
        if not ok:
            self.get_logger().warning("Failed to JPEG-encode annotated gesture frame.")
            return

        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.annotated_pub.publish(msg)

    def destroy_node(self):
        if getattr(self, "recognizer", None) is not None:
            self.recognizer.close()
            self.recognizer = None
        if self.show_window:
            try:
                cv2.destroyWindow("MediaPipe Gesture Recognizer")
            except cv2.error:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GestureRecognitionNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
