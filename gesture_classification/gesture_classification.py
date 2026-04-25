from ultralytics import YOLO
import torch
import cv2

class GestureNet(torch.nn.Module):
    def __init__(self, num_classes):
        super().__init__()
        self.fc = torch.nn.Sequential(
            torch.nn.Linear(42, 128),
            torch.nn.ReLU(),
            torch.nn.Linear(128, 64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, num_classes)
        )

    def forward(self, x):
        return self.fc(x)

# Load models
pose_model = YOLO("../models/hand_keypoints_trained.pt")
gesture_model = GestureNet(num_classes=4)
gesture_model.load_state_dict(torch.load("gesture_classifier.pth"))
gesture_model.eval()

GESTURES = ["open_hand", "fist", "point", "thumbs_up"]
cap = cv2.VideoCapture(0)

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

while True:
    ret, frame = cap.read()
    results = pose_model(frame, verbose=False)
    result = results[0]
    kps = result.keypoints.xy
    boxes = result.boxes.xyxy
    if kps is not None and boxes is not None and len(kps) > 0 and len(boxes) > 0:
        keypoints = kps[0].cpu().numpy()

        # Draw keypoints on the frame
        for (x_coord, y_coord) in keypoints:
            cv2.circle(frame, (int(x_coord), int(y_coord)), 3, (0, 255, 255), -1)

        pts = normalize_keypoints_to_bbox(keypoints, boxes[0].cpu().numpy())
        x = torch.tensor(pts, dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            pred = gesture_model(x).argmax(1).item()
        gesture = GESTURES[pred]
        cv2.putText(frame, gesture, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
    cv2.imshow("Gesture Recognition", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
