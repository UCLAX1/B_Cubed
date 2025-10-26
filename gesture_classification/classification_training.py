import torch
import torch.nn as nn
import torch.optim as optim
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder

# Load data
df = pd.read_csv("gesture_data.csv")
df = df[df["gesture"] != "gesture"]
X = df.drop(columns=["gesture"]).values.astype("float32")
y = LabelEncoder().fit_transform(df["gesture"])

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

model = GestureNet(num_classes=len(set(y)))
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=1e-3)

# Train loop
for epoch in range(100):
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