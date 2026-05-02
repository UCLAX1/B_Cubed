# Official MediaPipe Gesture Recognizer Training

This is the Google MediaPipe Model Maker path. It trains and exports a
`gesture_recognizer.task` bundle instead of using this repo's PyTorch CSV trainer.

## What Is Pretrained?

MediaPipe's pretrained gesture recognizer already detects these canned gestures:

- `Closed_Fist`
- `Open_Palm`
- `Pointing_Up`
- `Thumb_Up`
- `Thumb_Down`
- `Victory`
- `ILoveYou`
- `None`

For custom labels like `open_hand`, `fist`, `point`, and `thumbs_up`, Model Maker
still needs example images. It uses transfer learning and MediaPipe's hand detection
pipeline, so you are customizing Google's model rather than building the full hand
pipeline yourself.

## Install

```powershell
python -m pip install mediapipe-model-maker
```

If that fails in Python 3.13, create a Python 3.10 or 3.11 virtual environment just
for training. The exported `.task` file can be copied back and used by the live script.

## Option A: Test Google's Pretrained Canned Gestures

```powershell
python gesture_classification/gesture_recognizer_task_live.py --pretrained-if-missing
```

This does not use your custom labels. It uses Google's canned category names.

## Option B: Train Your Custom MediaPipe Gesture Recognizer

Collect image folders:

```powershell
python gesture_classification/model_maker_collection.py
```

The dataset must include a label named `none`. Use `none` for other hand poses that
should not trigger a command.

Train and export the official MediaPipe task bundle:

```powershell
python gesture_classification/model_maker_training.py
```

Run the exported model:

```powershell
python gesture_classification/gesture_recognizer_task_live.py
```
