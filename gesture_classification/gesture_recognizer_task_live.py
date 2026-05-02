import argparse
import os
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import mediapipe as mp

from mediapipe_static_hand import draw_hand


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CUSTOM_MODEL_PATH = os.path.join(BASE_DIR, "exported_model", "gesture_recognizer.task")
PRETRAINED_MODEL_PATH = os.path.join(BASE_DIR, "gesture_recognizer_pretrained.task")
PRETRAINED_GESTURE_RECOGNIZER_URL = (
    "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/"
    "gesture_recognizer/float16/1/gesture_recognizer.task"
)


def parse_args():
    parser = argparse.ArgumentParser(description="Run MediaPipe Gesture Recognizer task live.")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument(
        "--model-path",
        default=CUSTOM_MODEL_PATH,
        help="Path to an exported MediaPipe gesture_recognizer.task model.",
    )
    parser.add_argument(
        "--pretrained-if-missing",
        action="store_true",
        help="Use Google's canned pretrained gesture model if the custom model is missing.",
    )
    return parser.parse_args()


def ensure_pretrained_model():
    if os.path.isfile(PRETRAINED_MODEL_PATH):
        return PRETRAINED_MODEL_PATH

    print(f"Downloading pretrained MediaPipe Gesture Recognizer to: {PRETRAINED_MODEL_PATH}")
    try:
        urllib.request.urlretrieve(PRETRAINED_GESTURE_RECOGNIZER_URL, PRETRAINED_MODEL_PATH)
    except (OSError, urllib.error.URLError) as exc:
        raise RuntimeError(
            "Could not download the pretrained MediaPipe Gesture Recognizer. "
            f"Download it manually from {PRETRAINED_GESTURE_RECOGNIZER_URL} "
            f"and save it as {PRETRAINED_MODEL_PATH}."
        ) from exc

    return PRETRAINED_MODEL_PATH


def resolve_model_path(model_path, pretrained_if_missing):
    if os.path.isfile(model_path):
        return model_path
    if pretrained_if_missing:
        return ensure_pretrained_model()

    raise RuntimeError(
        f"Could not find custom MediaPipe model: {model_path}\n"
        "Train with gesture_classification/model_maker_training.py first, or run this script "
        "with --pretrained-if-missing to test Google's canned gesture recognizer."
    )


def create_recognizer(model_path):
    options = mp.tasks.vision.GestureRecognizerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=model_path),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=1,
    )
    return mp.tasks.vision.GestureRecognizer.create_from_options(options)


def first_gesture_name(result):
    if not result.gestures or not result.gestures[0]:
        return "None", 0.0
    category = result.gestures[0][0]
    return category.category_name, float(category.score)


def draw_first_hand(frame, result):
    if not result.hand_landmarks:
        return
    landmarks = result.hand_landmarks[0]
    points = np.array([[lm.x, lm.y, lm.z] for lm in landmarks], dtype=np.float32)
    draw_hand(frame, points)


def main():
    args = parse_args()
    model_path = resolve_model_path(args.model_path, args.pretrained_if_missing)
    print(f"Using MediaPipe gesture model: {model_path}")

    cap = cv2.VideoCapture(args.camera)
    start_time = time.monotonic()
    last_timestamp_ms = -1

    with create_recognizer(model_path) as recognizer:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
            timestamp_ms = int((time.monotonic() - start_time) * 1000)
            if timestamp_ms <= last_timestamp_ms:
                timestamp_ms = last_timestamp_ms + 1
            last_timestamp_ms = timestamp_ms

            result = recognizer.recognize_for_video(mp_image, timestamp_ms)
            gesture, confidence = first_gesture_name(result)
            draw_first_hand(frame, result)

            cv2.putText(
                frame,
                f"{gesture} {confidence:.2f}",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )
            cv2.imshow("MediaPipe Gesture Recognizer", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
