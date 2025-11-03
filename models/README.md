# YOLO Models Directory

This directory is for storing YOLO model files for human detection.

## Available Models

The project supports YOLOv5 and YOLOv8 models in ONNX format. Both models use 3D tensor outputs and require OpenCV 4.8+.

### YOLOv5 ONNX Model

The default model used is **yolov5n.onnx** (YOLOv5 nano):
- Size: ~11 MB
- Speed: ~30-60 FPS on CPU
- Accuracy: Sufficient for human detection
- Format: ONNX (compatible with OpenCV 4.8+)

### YOLOv8 ONNX Model

Also available: **yolov8n.onnx** (YOLOv8 nano):
- Size: ~13 MB
- Better accuracy than YOLOv5
- Format: ONNX (compatible with OpenCV 4.8+)

## Download Models

### Option 1: Direct Download

```bash
cd models/
# Download YOLOv8 nano ONNX model
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.onnx

# Or YOLOv5 nano ONNX model
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov5n.onnx
```

### Option 2: Convert from PyTorch

Convert PyTorch models to ONNX using ultralytics:

```python
from ultralytics import YOLO

# For YOLOv5
model = YOLO('yolov5n.pt')
model.export(format='onnx', simplify=True, opset=11, imgsz=640)

# For YOLOv8
model = YOLO('yolov8n.pt')
model.export(format='onnx', simplify=True, opset=11, imgsz=640)
```

### Option 3: Use Ultralytics CLI

```bash
# Install ultralytics if needed
pip install ultralytics

# Export YOLOv5 to ONNX
yolo export model=yolov5n.pt format=onnx simplify=True opset=11

# Or YOLOv8
yolo export model=yolov8n.pt format=onnx simplify=True opset=11
```

## Requirements

- **OpenCV 4.8+** required for YOLOv5/YOLOv8 models with 3D tensor outputs
- Both YOLOv5 and YOLOv8 models use output format: `[1, 84, 8400]`
- Models support human detection (class ID 0 in COCO dataset)

## Recommended Model

For real-time robotics applications, use **yolov5n.onnx** (nano):
- Size: ~11 MB
- Speed: ~30-60 FPS on CPU
- Accuracy: Sufficient for human detection
- Format: ONNX (compatible with OpenCV 4.8+)

For better accuracy, use **yolov8n.onnx**:
- Better detection accuracy than YOLOv5
- Slightly larger file size (~13 MB)

## License

YOLOv8 models are licensed under AGPL-3.0.
Free for academic and educational use.

For commercial use, contact Ultralytics for licensing.

## Model Files (gitignored)

Model files are not committed to git (too large).

