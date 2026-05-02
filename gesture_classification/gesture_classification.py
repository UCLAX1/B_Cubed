from gesture_recognizer_task_live import main


# The old self-trained PyTorch classifier path is intentionally disabled.
# Use classification_collection.py and classification_training.py only if you
# want to go back to the repo's custom CSV/PyTorch training flow.
#
# This entry point now uses Google's MediaPipe Gesture Recognizer task model.
# By default, gesture_recognizer_task_live.py looks for:
#   gesture_classification/exported_model/gesture_recognizer.task
# Add --pretrained-if-missing to use Google's canned pretrained gestures.


if __name__ == "__main__":
    main()
