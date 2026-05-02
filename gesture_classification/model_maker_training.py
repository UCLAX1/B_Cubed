import argparse
import os


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DATASET_DIR = os.path.join(BASE_DIR, "model_maker_dataset")
DEFAULT_EXPORT_DIR = os.path.join(BASE_DIR, "exported_model")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Train an official MediaPipe Model Maker gesture recognizer."
    )
    parser.add_argument("--dataset-dir", default=DEFAULT_DATASET_DIR)
    parser.add_argument("--export-dir", default=DEFAULT_EXPORT_DIR)
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--learning-rate", type=float, default=0.001)
    parser.add_argument("--min-detection-confidence", type=float, default=0.6)
    return parser.parse_args()


def require_model_maker():
    try:
        from mediapipe_model_maker import gesture_recognizer
    except ImportError as exc:
        raise RuntimeError(
            "MediaPipe Model Maker is not installed. Install it with:\n"
            "python -m pip install mediapipe-model-maker\n\n"
            "If pip cannot install it in this Python version, create a Python 3.10 "
            "or 3.11 virtual environment for training, then copy the exported "
            "gesture_recognizer.task back into this project."
        ) from exc
    return gesture_recognizer


def validate_dataset(dataset_dir):
    if not os.path.isdir(dataset_dir):
        raise RuntimeError(f"Dataset directory does not exist: {dataset_dir}")

    labels = [
        name
        for name in os.listdir(dataset_dir)
        if os.path.isdir(os.path.join(dataset_dir, name))
    ]
    if "none" not in labels:
        raise RuntimeError("MediaPipe Model Maker requires a label directory named 'none'.")

    empty_labels = []
    for label in labels:
        label_dir = os.path.join(dataset_dir, label)
        images = [
            name
            for name in os.listdir(label_dir)
            if name.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".webp"))
        ]
        if not images:
            empty_labels.append(label)
    if empty_labels:
        raise RuntimeError(f"These labels have no images: {empty_labels}")

    return labels


def main():
    args = parse_args()
    gesture_recognizer = require_model_maker()
    labels = validate_dataset(args.dataset_dir)
    print(f"Training labels: {labels}")

    data = gesture_recognizer.Dataset.from_folder(
        dirname=args.dataset_dir,
        hparams=gesture_recognizer.HandDataPreprocessingParams(
            min_detection_confidence=args.min_detection_confidence
        ),
    )
    train_data, rest_data = data.split(0.8)
    validation_data, test_data = rest_data.split(0.5)

    hparams = gesture_recognizer.HParams(
        export_dir=args.export_dir,
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
    )
    model_options = gesture_recognizer.ModelOptions(
        dropout_rate=0.15,
        layer_widths=[128, 64],
    )
    options = gesture_recognizer.GestureRecognizerOptions(
        model_options=model_options,
        hparams=hparams,
    )

    model = gesture_recognizer.GestureRecognizer.create(
        train_data=train_data,
        validation_data=validation_data,
        options=options,
    )

    loss, accuracy = model.evaluate(test_data, batch_size=1)
    print(f"Test loss: {loss:.4f}, Test accuracy: {accuracy:.4f}")
    model.export_model()
    print(f"Exported MediaPipe task model to: {os.path.join(args.export_dir, 'gesture_recognizer.task')}")


if __name__ == "__main__":
    main()
