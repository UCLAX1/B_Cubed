from ultralytics import YOLO
model = YOLO("hand_gestures.pt")
results = model.predict(source=0, show=True)