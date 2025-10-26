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

while True:
    ret, frame = cap.read()
    results = pose_model(frame, verbose=False)
    kps = results[0].keypoints.xy
    if kps is not None and len(kps) > 0:
        pts = kps[0].cpu().numpy().flatten()
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
