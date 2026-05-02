import os
import time
import urllib.error
import urllib.request

import numpy as np

try:
    import mediapipe as mp
except ImportError as exc:
    mp = None
    _MEDIAPIPE_IMPORT_ERROR = exc


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_HAND_LANDMARKER_MODEL = os.path.join(BASE_DIR, "hand_landmarker.task")
HAND_LANDMARKER_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/1/hand_landmarker.task"
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
            "MediaPipe is not installed. Install it with: python -m pip install mediapipe"
        ) from _MEDIAPIPE_IMPORT_ERROR
    return mp


def ensure_hand_landmarker_model(model_path=DEFAULT_HAND_LANDMARKER_MODEL):
    if os.path.isfile(model_path):
        return model_path

    os.makedirs(os.path.dirname(model_path), exist_ok=True)
    print(f"Downloading MediaPipe hand landmarker model to: {model_path}")
    try:
        urllib.request.urlretrieve(HAND_LANDMARKER_URL, model_path)
    except (OSError, urllib.error.URLError) as exc:
        raise RuntimeError(
            "Could not download the MediaPipe hand landmarker model. "
            f"Download it manually from {HAND_LANDMARKER_URL} and save it as {model_path}."
        ) from exc

    return model_path


class TasksHands:
    def __init__(self, landmarker):
        self.landmarker = landmarker
        self.start_time = time.monotonic()
        self.last_timestamp_ms = -1

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def close(self):
        self.landmarker.close()

    def detect_frame(self, frame_bgr):
        import cv2

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        timestamp_ms = int((time.monotonic() - self.start_time) * 1000)
        if timestamp_ms <= self.last_timestamp_ms:
            timestamp_ms = self.last_timestamp_ms + 1
        self.last_timestamp_ms = timestamp_ms

        result = self.landmarker.detect_for_video(mp_image, timestamp_ms)
        if not result.hand_landmarks:
            return None, None

        landmarks = result.hand_landmarks[0]
        handedness = None
        if result.handedness and result.handedness[0]:
            handedness = result.handedness[0][0].category_name

        points = np.array(
            [[lm.x, lm.y, lm.z] for lm in landmarks],
            dtype=np.float32,
        )
        return points, handedness


def create_hands(
    max_num_hands=1,
    min_detection_confidence=0.6,
    min_tracking_confidence=0.6,
    model_path=DEFAULT_HAND_LANDMARKER_MODEL,
):
    mediapipe = require_mediapipe()
    if hasattr(mediapipe, "solutions"):
        return mediapipe.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

    model_path = ensure_hand_landmarker_model(model_path)
    options = mediapipe.tasks.vision.HandLandmarkerOptions(
        base_options=mediapipe.tasks.BaseOptions(model_asset_path=model_path),
        running_mode=mediapipe.tasks.vision.RunningMode.VIDEO,
        num_hands=max_num_hands,
        min_hand_detection_confidence=min_detection_confidence,
        min_hand_presence_confidence=min_detection_confidence,
        min_tracking_confidence=min_tracking_confidence,
    )
    landmarker = mediapipe.tasks.vision.HandLandmarker.create_from_options(options)
    return TasksHands(landmarker)


def detect_first_hand(frame_bgr, hands):
    if hasattr(hands, "detect_frame"):
        return hands.detect_frame(frame_bgr)

    import cv2

    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    frame_rgb.flags.writeable = False
    results = hands.process(frame_rgb)
    frame_rgb.flags.writeable = True

    if not results.multi_hand_landmarks:
        return None, None

    landmarks = results.multi_hand_landmarks[0]
    handedness = None
    if results.multi_handedness:
        handedness = results.multi_handedness[0].classification[0].label

    points = np.array(
        [[lm.x, lm.y, lm.z] for lm in landmarks.landmark],
        dtype=np.float32,
    )
    return points, handedness


def landmarks_to_features(landmarks):
    pts = np.asarray(landmarks, dtype=np.float32).copy()
    if pts.shape != (21, 3):
        raise ValueError(f"Expected MediaPipe landmarks with shape (21, 3), got {pts.shape}.")

    pts -= pts[0]
    scale = float(np.max(np.linalg.norm(pts[:, :2], axis=1)))
    if scale < 1e-6:
        scale = 1.0
    pts /= scale
    return pts.reshape(-1).astype(np.float32)


def draw_hand(frame_bgr, landmarks, color=(0, 255, 0)):
    import cv2

    h, w = frame_bgr.shape[:2]
    pts = np.asarray(landmarks, dtype=np.float32)
    pixels = np.column_stack((pts[:, 0] * w, pts[:, 1] * h)).astype(int)

    for a, b in HAND_EDGES:
        cv2.line(frame_bgr, tuple(pixels[a]), tuple(pixels[b]), color, 2)
    for x, y in pixels:
        cv2.circle(frame_bgr, (int(x), int(y)), 3, (0, 255, 255), -1)

    return frame_bgr
