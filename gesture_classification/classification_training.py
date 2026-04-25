import torch
import torch.nn as nn
import torch.optim as optim
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
import joblib

DATA_PATH = "gesture_data_normalized.csv"
GESTURES = np.array(["open_hand", "fist", "point", "thumbs_up"])

# Load data
df = pd.read_csv(DATA_PATH)
df = df[df["gesture"] != "gesture"]
unknown_gestures = sorted(set(df["gesture"]) - set(GESTURES))
if unknown_gestures:
    raise RuntimeError(f"{DATA_PATH} contains unknown gestures: {unknown_gestures}")

X = df.drop(columns=["gesture"]).values.astype("float32")

if X.shape[1] != 42:
    raise RuntimeError(f"Expected 42 normalized keypoint features, got {X.shape[1]}.")
if not np.isfinite(X).all():
    raise RuntimeError(f"{DATA_PATH} contains non-finite values.")
if np.abs(X).max() > 2.0:
    raise RuntimeError(
        f"{DATA_PATH} looks like it contains raw pixel coordinates. "
        "Recollect data with classification_collection.py so every row is bbox-normalized."
    )

le = LabelEncoder()
le.classes_ = GESTURES
y = le.transform(df["gesture"])
joblib.dump(le, "gesture_labels.pkl")

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)

# Convert to torch tensors
X_train = torch.tensor(X_train)
y_train = torch.tensor(y_train)
X_test = torch.tensor(X_test)
y_test = torch.tensor(y_test)

class GestureNet(nn.Module):
    def __init__(self, num_classes):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(42, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, num_classes)
        )

    def forward(self, x):
        return self.fc(x)

model = GestureNet(num_classes=len(GESTURES))
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=1e-3)

# Train loop
for epoch in range(1000):
    optimizer.zero_grad()
    outputs = model(X_train)
    loss = criterion(outputs, y_train)
    loss.backward()
    optimizer.step()

    if epoch % 5 == 0:
        with torch.no_grad():
            acc = (outputs.argmax(1) == y_train).float().mean()
        print(f"Epoch {epoch}, Loss: {loss.item():.4f}, Train Acc: {acc.item():.3f}")

torch.save(model.state_dict(), "gesture_classifier.pth")
