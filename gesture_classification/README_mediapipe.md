# MediaPipe Static Gesture Training

This pipeline uses Google MediaPipe Hands to detect 21 hand landmarks, converts each
static hand pose into 63 normalized features, trains a small PyTorch classifier, and
runs live webcam inference.

## Setup

```powershell
python -m pip install -r gesture_classification/requirements.txt
```

## Collect Samples

```powershell
python gesture_classification/classification_collection.py
```

Hold one static gesture at a time. Press `n` to move to the next label and `q` to stop.
The default labels are:

- `open_hand`
- `fist`
- `point`
- `thumbs_up`

Collect varied samples for each label: different distances, rotations, lighting, and
both hands if you plan to use both hands.

## Train

```powershell
python gesture_classification/classification_training.py
```

This creates:

- `gesture_classification/gesture_classifier_mediapipe.pth`
- `gesture_classification/gesture_classifier_mediapipe_meta.json`
- `gesture_classification/gesture_labels_mediapipe.pkl`

## Run

```powershell
python gesture_classification/gesture_classification.py
```

Press `q` to stop.
