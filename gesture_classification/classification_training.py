import argparse
import json
import os

import joblib
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder

from gesture_model import GestureNet


GESTURES = np.array(["open_hand", "fist", "point", "thumbs_up"])
BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def parse_args():
    parser = argparse.ArgumentParser(description="Train a static hand gesture classifier.")
    parser.add_argument("--data-path", default=os.path.join(BASE_DIR, "gesture_data_mediapipe.csv"))
    parser.add_argument("--model-path", default=os.path.join(BASE_DIR, "gesture_classifier_mediapipe.pth"))
    parser.add_argument("--labels-path", default=os.path.join(BASE_DIR, "gesture_labels_mediapipe.pkl"))
    parser.add_argument("--meta-path", default=os.path.join(BASE_DIR, "gesture_classifier_mediapipe_meta.json"))
    parser.add_argument("--detector", choices=["mediapipe", "yolo"], default="mediapipe")
    parser.add_argument("--epochs", type=int, default=400)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--test-size", type=float, default=0.2)
    return parser.parse_args()


def load_dataset(data_path, expected_features):
    df = pd.read_csv(data_path)
    df = df[df["gesture"] != "gesture"]

    unknown_gestures = sorted(set(df["gesture"]) - set(GESTURES))
    if unknown_gestures:
        raise RuntimeError(f"{data_path} contains unknown gestures: {unknown_gestures}")

    missing_gestures = sorted(set(GESTURES) - set(df["gesture"]))
    if missing_gestures:
        raise RuntimeError(f"{data_path} has no samples for: {missing_gestures}")

    X = df.drop(columns=["gesture"]).values.astype("float32")
    if X.shape[1] != expected_features:
        raise RuntimeError(f"Expected {expected_features} landmark features, got {X.shape[1]}.")
    if not np.isfinite(X).all():
        raise RuntimeError(f"{data_path} contains non-finite values.")

    le = LabelEncoder()
    le.classes_ = GESTURES
    y = le.transform(df["gesture"])
    return X, y, le


def main():
    args = parse_args()
    expected_features = 63 if args.detector == "mediapipe" else 42
    X, y, label_encoder = load_dataset(args.data_path, expected_features)
    joblib.dump(label_encoder, args.labels_path)

    X_train, X_test, y_train, y_test = train_test_split(
        X,
        y,
        test_size=args.test_size,
        stratify=y,
        random_state=42,
    )

    X_train = torch.tensor(X_train)
    y_train = torch.tensor(y_train)
    X_test = torch.tensor(X_test)
    y_test = torch.tensor(y_test)

    model = GestureNet(input_size=expected_features, num_classes=len(GESTURES))
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=args.learning_rate)

    for epoch in range(args.epochs):
        model.train()
        optimizer.zero_grad()
        outputs = model(X_train)
        loss = criterion(outputs, y_train)
        loss.backward()
        optimizer.step()

        if epoch % 25 == 0 or epoch == args.epochs - 1:
            model.eval()
            with torch.no_grad():
                train_acc = (outputs.argmax(1) == y_train).float().mean().item()
                test_outputs = model(X_test)
                test_acc = (test_outputs.argmax(1) == y_test).float().mean().item()
            print(
                f"Epoch {epoch:04d}, Loss: {loss.item():.4f}, "
                f"Train Acc: {train_acc:.3f}, Test Acc: {test_acc:.3f}"
            )

    torch.save(model.state_dict(), args.model_path)
    with open(args.meta_path, "w") as f:
        json.dump(
            {
                "detector": args.detector,
                "feature_count": expected_features,
                "data_path": args.data_path,
                "gestures": GESTURES.tolist(),
                "model_class": "GestureNet",
                "normalization": "wrist_centered_max_xy_scale" if args.detector == "mediapipe" else "bbox",
            },
            f,
            indent=2,
        )

    print(f"Saved model to {args.model_path}")
    print(f"Saved labels to {args.labels_path}")
    print(f"Saved metadata to {args.meta_path}")


if __name__ == "__main__":
    main()
