import argparse
import csv
import os

import cv2

from mediapipe_static_hand import detect_first_hand, draw_hand, create_hands, landmarks_to_features


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
GESTURES = ["open_hand", "fist", "point", "thumbs_up"]


def parse_args():
    parser = argparse.ArgumentParser(description="Collect static hand gesture landmark samples.")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--detector", choices=["mediapipe", "yolo"], default="mediapipe")
    parser.add_argument("--save-path", default=os.path.join(BASE_DIR, "gesture_data_mediapipe.csv"))
    parser.add_argument("--yolo-model", default=os.path.join(BASE_DIR, "..", "models", "hand_keypoints_trained.pt"))
    return parser.parse_args()


def write_header_if_needed(writer, save_path, detector):
    if os.path.isfile(save_path) and os.path.getsize(save_path) > 0:
        return

    if detector == "mediapipe":
        columns = [coord for i in range(21) for coord in (f"x{i}", f"y{i}", f"z{i}")]
    else:
        columns = [coord for i in range(21) for coord in (f"x{i}", f"y{i}")]
    writer.writerow(["gesture"] + columns)


def normalize_yolo_keypoints_to_bbox(keypoints, bbox):
    x1, y1, x2, y2 = bbox
    bw = max(1.0, float(x2 - x1))
    bh = max(1.0, float(y2 - y1))
    cx = (float(x1) + float(x2)) * 0.5
    cy = (float(y1) + float(y2)) * 0.5

    pts = keypoints.copy()
    pts[:, 0] = (pts[:, 0] - cx) / bw
    pts[:, 1] = (pts[:, 1] - cy) / bh
    return pts.flatten()


def collect_with_mediapipe(args):
    cap = cv2.VideoCapture(args.camera)
    current_label = 0
    print(f"Recording gesture: {GESTURES[current_label]}")
    print(f"Saving MediaPipe landmarks to: {args.save_path}")
    print("Hold a static gesture. Press n for next label, q to stop.")

    with create_hands() as hands, open(args.save_path, "a", newline="") as f:
        writer = csv.writer(f)
        write_header_if_needed(writer, args.save_path, "mediapipe")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            landmarks, handedness = detect_first_hand(frame, hands)
            if landmarks is not None:
                features = landmarks_to_features(landmarks)
                writer.writerow([GESTURES[current_label]] + features.tolist())
                draw_hand(frame, landmarks)
                if handedness:
                    cv2.putText(frame, handedness, (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.putText(
                frame,
                f"Gesture: {GESTURES[current_label]} | n next | q stop",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.imshow("Collecting MediaPipe Gestures", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("n"):
                current_label = (current_label + 1) % len(GESTURES)
                print(f"Now recording: {GESTURES[current_label]}")
            elif key == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


def collect_with_yolo(args):
    from ultralytics import YOLO

    model = YOLO(args.yolo_model)
    cap = cv2.VideoCapture(args.camera)
    current_label = 0
    print(f"Recording gesture: {GESTURES[current_label]}")
    print(f"Saving YOLO bbox-normalized keypoints to: {args.save_path}")

    with open(args.save_path, "a", newline="") as f:
        writer = csv.writer(f)
        write_header_if_needed(writer, args.save_path, "yolo")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            result = model(frame, verbose=False)[0]
            keypoints = result.keypoints.xy
            boxes = result.boxes.xyxy
            if keypoints is not None and boxes is not None and len(keypoints) > 0 and len(boxes) > 0:
                pts = normalize_yolo_keypoints_to_bbox(keypoints[0].cpu().numpy(), boxes[0].cpu().numpy())
                writer.writerow([GESTURES[current_label]] + pts.tolist())

            cv2.putText(
                frame,
                f"Gesture: {GESTURES[current_label]} | n next | q stop",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.imshow("Collecting YOLO Gestures", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("n"):
                current_label = (current_label + 1) % len(GESTURES)
                print(f"Now recording: {GESTURES[current_label]}")
            elif key == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


def main():
    args = parse_args()
    if args.detector == "mediapipe":
        collect_with_mediapipe(args)
    else:
        collect_with_yolo(args)


if __name__ == "__main__":
    main()
