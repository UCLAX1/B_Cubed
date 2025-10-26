from ultralytics import YOLO
import cv2
import numpy as np
import csv

model = YOLO("../models/hand_keypoints_trained.pt")  # your trained model
cap = cv2.VideoCapture(0)

GESTURES = ["open_hand", "fist", "point", "thumbs_up"]
SAVE_PATH = "gesture_data.csv"

current_label = 0  # index into GESTURES
print(f"Recording gesture: {GESTURES[current_label]}")

with open(SAVE_PATH, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["gesture"] + [f"x{i}" for i in range(21)] + [f"y{i}" for i in range(21)])

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame)
        keypoints = results[0].keypoints.xy

        if keypoints is not None and len(keypoints) > 0:
            pts = keypoints[0].cpu().numpy().flatten()  # (42,)
            writer.writerow([GESTURES[current_label]] + pts.tolist())

        cv2.putText(frame, f"Gesture: {GESTURES[current_label]} (press n to next)", (10, 40),
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
