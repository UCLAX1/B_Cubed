import argparse
import os
import time

import cv2


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DATASET_DIR = os.path.join(BASE_DIR, "model_maker_dataset")
DEFAULT_LABELS = ["open_hand", "fist", "point", "thumbs_up", "none"]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect image folders for MediaPipe Model Maker gesture training."
    )
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--dataset-dir", default=DEFAULT_DATASET_DIR)
    parser.add_argument("--labels", nargs="+", default=DEFAULT_LABELS)
    parser.add_argument("--capture-interval", type=float, default=0.25)
    return parser.parse_args()


def ensure_label_dirs(dataset_dir, labels):
    for label in labels:
        os.makedirs(os.path.join(dataset_dir, label), exist_ok=True)


def next_image_path(dataset_dir, label):
    label_dir = os.path.join(dataset_dir, label)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    millis = int((time.time() % 1.0) * 1000)
    return os.path.join(label_dir, f"{timestamp}_{millis:03d}.jpg")


def main():
    args = parse_args()
    if "none" not in args.labels:
        raise RuntimeError(
            "MediaPipe Model Maker requires a label named 'none'. "
            "Use it for hand poses that are not one of your command gestures."
        )

    ensure_label_dirs(args.dataset_dir, args.labels)
    current_label = 0
    last_capture = 0.0
    saved_count = 0

    cap = cv2.VideoCapture(args.camera)
    print(f"Saving images to: {args.dataset_dir}")
    print("Press n for next label, space to pause/resume capture, q to stop.")
    print(f"Current label: {args.labels[current_label]}")
    capturing = True

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        now = time.monotonic()
        label = args.labels[current_label]
        if capturing and now - last_capture >= args.capture_interval:
            image_path = next_image_path(args.dataset_dir, label)
            cv2.imwrite(image_path, frame)
            saved_count += 1
            last_capture = now

        status = "REC" if capturing else "PAUSED"
        cv2.putText(
            frame,
            f"{status} | Label: {label} | saved: {saved_count}",
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            frame,
            "n next | space pause | q stop",
            (10, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )
        cv2.imshow("Collect MediaPipe Model Maker Images", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("n"):
            current_label = (current_label + 1) % len(args.labels)
            print(f"Current label: {args.labels[current_label]}")
        elif key == ord(" "):
            capturing = not capturing
            print("Capturing" if capturing else "Paused")
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
