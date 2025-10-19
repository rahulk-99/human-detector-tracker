# YOLO Models Directory

This directory is for storing YOLO model files.

## Phase 0 (Current)

Phase 0 uses mock/stub implementations, so no actual model files are required.

## Phase 1 (Future)

Download YOLOv8 ONNX model for Phase 1:

```bash
# Download YOLOv8 nano model (smallest, fastest)
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.onnx

# Or download YOLOv8 small model (better accuracy)
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.onnx

# Or download YOLOv8 medium model (balanced)
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8m.onnx
```

## Recommended Model

For real-time robotics applications, use **yolov8n.onnx** (nano):
- Size: ~6 MB
- Speed: ~30-60 FPS on CPU
- Accuracy: Sufficient for human detection

## License

YOLOv8 models are licensed under AGPL-3.0.
Free for academic and educational use.

For commercial use, contact Ultralytics for licensing.

## Model Files (gitignored)

Model files are not committed to git (too large).
Add `.onnx`, `.pt`, and `.weights` files to this directory locally.

