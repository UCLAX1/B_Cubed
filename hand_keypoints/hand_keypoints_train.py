from ultralytics import YOLO

def main():
    model = YOLO("../models/yolo11n-pose.pt")

    results = model.train(
        data="hand-keypoints.yaml",
        epochs=100,
        imgsz=640,
        device="0",
        batch=48,
        workers=8,  # i7-13700K can handle 12 threads fine
    )


if __name__ == "__main__":
    main()

