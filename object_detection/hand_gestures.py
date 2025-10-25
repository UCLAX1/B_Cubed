from ultralytics import YOLO


model = YOLO("yolo11n-pose.pt")

results = model.train(data="hand-keypoints.yaml", epochs=100, imgsz=640, device = "mps")
