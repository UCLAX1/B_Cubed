from ultralytics import YOLO
import cv2
import csv
import os

model = YOLO("../models/hand_keypoints_trained.pt")  # your trained model
cap = cv2.VideoCapture(0)

GESTURES = ["open_hand", "fist", "point", "thumbs_up"]
SAVE_PATH = "gesture_data_normalized.csv"
file_exists = os.path.isfile(SAVE_PATH)

def validate_existing_normalized_data(path):
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return
        if len(header) != 43:
            raise RuntimeError(f"{path} must have 1 gesture column and 42 keypoint columns.")

        for row_num, row in enumerate(reader, start=2):
            if not row or row[0] == "gesture":
                continue
            values = [float(value) for value in row[1:]]
            if len(values) != 42:
                raise RuntimeError(f"{path}:{row_num} does not contain 42 keypoint values.")
            if max(abs(value) for value in values) > 2.0:
                raise RuntimeError(
                    f"{path}:{row_num} looks like raw pixel data. "
                    "Start a fresh normalized CSV before collecting more samples."
                )

def normalize_keypoints_to_bbox(keypoints, bbox):
    x1, y1, x2, y2 = bbox
    bw = max(1.0, float(x2 - x1))
    bh = max(1.0, float(y2 - y1))
    cx = (float(x1) + float(x2)) * 0.5
    cy = (float(y1) + float(y2)) * 0.5

    pts = keypoints.copy()
    pts[:, 0] = (pts[:, 0] - cx) / bw
    pts[:, 1] = (pts[:, 1] - cy) / bh
    return pts.flatten()


current_label = 0  # index into GESTURES
print(f"Recording gesture: {GESTURES[current_label]}")
print(f"Saving normalized keypoints to: {SAVE_PATH}")

if file_exists:
    validate_existing_normalized_data(SAVE_PATH)

with open(SAVE_PATH, "a", newline="") as f:
    writer = csv.writer(f)
    if not file_exists:
        columns = [coord for i in range(21) for coord in (f"x{i}", f"y{i}")]
        writer.writerow(["gesture"] + columns)



    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame, verbose=False)
        result = results[0]
        keypoints = result.keypoints.xy
        boxes = result.boxes.xyxy

        if keypoints is not None and boxes is not None and len(keypoints) > 0 and len(boxes) > 0:
            pts = normalize_keypoints_to_bbox(
                keypoints[0].cpu().numpy(),
                boxes[0].cpu().numpy()
            )
            writer.writerow([GESTURES[current_label]] + pts.tolist())

        cv2.putText(frame, f"Gesture: {GESTURES[current_label]} (press n to next)(press q to stop)", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Collecting", frame)

        key = cv2.waitKey(1)
        if key == ord('n'):
            current_label = (current_label + 1) % len(GESTURES)
            print(f"Now recording: {GESTURES[current_label]}")
        elif key == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
