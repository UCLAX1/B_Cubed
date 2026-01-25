import torch

class GestureNet(torch.nn.Module):
    def __init__(self, num_classes=4):
        super().__init__()
        self.fc = torch.nn.Sequential(
            torch.nn.Linear(42, 128),
            torch.nn.ReLU(),
            torch.nn.Linear(128, 64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, num_classes)
        )

    def forward(self, x):
        return self.fc(x)

# Load PyTorch model
model = GestureNet(num_classes=4)
state = torch.load("gesture_classifier.pth", map_location="cpu")
model.load_state_dict(state)
model.eval()

# Dummy input: (batch=1, features=42)
dummy = torch.randn(1, 42, dtype=torch.float32)

# Export
torch.onnx.export(
    model,
    dummy,
    "gesture_classifier.onnx",
    opset_version=18,
    input_names=["input"],
    output_names=["logits"],
    dynamic_axes={
        "input": {0: "batch"},
        "logits": {0: "batch"},
    },
)
print("Saved gesture_classifier.onnx")