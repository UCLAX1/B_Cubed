from ultralytics import YOLO
model = YOLO("../models/hand_keypoints_trained.pt")
results = model.predict(source=0, show=True)