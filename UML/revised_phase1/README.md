# Revised UML Diagrams - Phase 1

This directory contains the **revised UML diagrams** reflecting the Phase 1 implementation with real YOLO and Kalman Filter algorithms.

## 📊 Diagrams Included

### 1. Class Diagram (Revised)
**File**: `class_diagram_revised.puml`

**Key Changes from Phase 0**:
- ✅ **YOLODetector**: Real OpenCV DNN integration
  - Added `cv::dnn::Net net_` (OpenCV network)
  - Added `classNames_` (COCO dataset classes)
  - Added `colors_` for visualization
  - Real preprocessing, postprocessing, NMS

- ✅ **KalmanFilter**: Full matrix operations
  - Proper state vector with covariance
  - Matrix multiplication functions
  - Matrix inversion (Gauss-Jordan)
  - Matrix transpose and addition
  - Complete Kalman equations

- ✅ **KalmanTracker**: Integration enhanced
  - Actual Kalman filter usage
  - Real prediction and update cycles
  - Proper state management

### 2. Sequence Diagram (Revised)
**File**: `sequence_diagram_revised.puml`

**Key Changes from Phase 0**:
- ✅ Real YOLO inference flow
  - OpenCV Mat conversion
  - Blob creation
  - `net_->forward()` call
  - Parse YOLO output format

- ✅ Complete Kalman operations
  - Matrix multiply for state prediction
  - Matrix operations for covariance
  - Innovation computation
  - Kalman gain calculation
  - State and covariance updates

- ✅ Enhanced coordinate transformation
  - Real depth estimation
  - Pinhole camera model math

### 3. Activity Diagram (Revised)
**File**: `activity_diagram_revised.puml`

**Key Changes from Phase 0**:
- ✅ Real image preprocessing (resize, normalize, RGB conversion)
- ✅ Actual YOLO inference with OpenCV DNN
- ✅ Real NMS algorithm with IoU calculations
- ✅ Full matrix operations for Kalman filter
- ✅ Proper covariance propagation
- ✅ Innovation and gain computations
- ✅ Real velocity estimation

## 🔄 Comparison: Phase 0 vs Phase 1

| Aspect | Phase 0 (Initial) | Phase 1 (Revised) |
|--------|------------------|-------------------|
| **YOLO** | Mock detections | Real OpenCV DNN inference |
| **Kalman** | Simplified math | Full matrix operations |
| **Preprocessing** | Stub | Real image transformations |
| **NMS** | Basic | Complete IoU algorithm |
| **State** | Basic update | Full covariance tracking |
| **Matrix Ops** | None | Multiply, inverse, transpose |
| **Depth** | Simple estimate | Pinhole camera model |

## 📈 Metrics

**Code Added in Phase 1**:
- YOLODetector.cpp: **+383 lines** (real implementation)
- KalmanFilter.cpp: **+351 lines** (matrix operations)
- KalmanTracker.cpp: **+86 lines** (integration)
- **Total**: **~820 lines of real algorithms**

## 🎯 What Makes Phase 1 "Real"

### YOLODetector
```cpp
// Phase 0: Mock
return mockDetections;

// Phase 1: Real
cv::Mat blob;
cv::dnn::blobFromImage(image, blob, 1/255.0, cv::Size(640,640));
net_->setInput(blob);
std::vector<cv::Mat> outputs;
net_->forward(outputs);
// Parse and filter real detections
```

### KalmanFilter
```cpp
// Phase 0: Simplified
state_[i] += velocity_[i] * dt;

// Phase 1: Real
// Predict: x_k|k-1 = F * x_k-1|k-1
matrixMultiply(F_, state_, P_pred, ...);
// Update: x_k = x + K*(z - H*x)
KalmanGain K = P*H^T * inv(H*P*H^T + R);
```

## 📸 Generate Images

### Generate PNG Images

```bash
# Install PlantUML (if not installed)
sudo apt-get install plantuml graphviz

# Generate images from PlantUML files
plantuml UML/revised/*.puml

# This creates:
# - class_diagram_revised.png
# - sequence_diagram_revised.png
# - activity_diagram_revised.png
```

### Or Use Online Editor

1. Go to http://www.plantuml.com/plantuml/uml/
2. Copy content from `.puml` files
3. Paste and generate
4. Download as PNG or SVG

## 📝 File Locations

- **PlantUML Sources**: `UML/revised/*.puml`
- **Generated Images**: `UML/revised/*.png` (after running plantuml)
- **Phase 0 Diagrams**: `UML/initial/*.puml`

## 🔗 Usage

### In Documentation
Update README.md to reference revised diagrams:

```markdown
## UML Diagrams (Revised for Phase 1)

![Revised Class Diagram](UML/revised/class_diagram_revised.png)
![Revised Sequence Diagram](UML/revised/sequence_diagram_revised.png)
![Revised Activity Diagram](UML/revised/activity_diagram_revised.png)
```

### In Presentations
Use these diagrams to show:
- Real implementation progress
- Matrix operation details
- OpenCV integration
- Algorithm complexity

---

**Created**: Phase 1  
**Last Updated**: October 27, 2025  
**Team**: Group 2 - Acme Robotics

