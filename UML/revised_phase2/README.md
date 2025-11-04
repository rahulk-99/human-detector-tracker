# Revised UML Diagrams - Phase 2

This directory contains the **revised UML diagrams** reflecting the Phase 2 implementation with video processing, visualization, and code optimization.

## 📊 Diagrams Included

### 1. Class Diagram (Revised Phase 2)
**File**: `class_diagram_revised.puml`

**Key Changes from Phase 1**:
- ✅ **PerceptionPipeline**: Enhanced with Phase 2 features
  - Added `processVideo()` method for video file processing
  - Added `visualizeFrame()` method for OpenCV visualization
  - Added `setVisualization()` and `isVisualizationEnabled()` methods
  - Removed `processCamera()` (video-only application)
  - Added `getCameraModel()` getter

- ✅ **YOLODetector**: Optimized implementation
  - Removed unused color generation
  - Removed PyTorch loading fallback
  - Removed YOLOv5 format support
  - Optimized for human-only detection (YOLOv8 ONNX)

- ✅ **Detection**: Added setters
  - `setBoundingBox()`, `setConfidence()`, `setClass()`

- ✅ **Track**: Enhanced getters
  - `getTimestamp()`, `getBoundingBox()` methods

- ✅ **All Classes**: Complete method coverage
  - All public methods documented
  - Removed redundant test-only methods

### 2. Sequence Diagram (Revised Phase 2)
**File**: `sequence_diagram_revised.puml`

**Key Changes from Phase 1**:
- ✅ **Video Processing Flow**
  - Open video file with VideoCapture
  - Frame-by-frame processing loop
  - FPS extraction and timestamp calculation
  - Results collection for all frames

- ✅ **Visualization Integration**
  - Conditional visualization based on flag
  - OpenCV Mat conversion
  - Bounding box drawing for detections
  - Track ID and state visualization
  - Frame information display (FPS, count)

- ✅ **Complete Workflow**
  - Shows full video processing pipeline
  - Detection → Tracking → Visualization flow

### 3. Activity Diagram (Revised Phase 2)
**File**: `activity_diagram_revised.puml`

**Key Changes from Phase 1**:
- ✅ **Video Processing Path**
  - Video file opening and FPS extraction
  - Frame reading loop
  - Timestamp calculation from FPS

- ✅ **Visualization Branch**
  - Conditional visualization path
  - OpenCV drawing operations
  - Window display with cv::imshow

- ✅ **Enhanced Flow**
  - Complete decision tree for video vs single frame
  - Visualization decision branches
  - Track state management (TENTATIVE → CONFIRMED → LOST)

## 🔄 Comparison: Phase 1 vs Phase 2

| Aspect | Phase 1 | Phase 2 |
|--------|---------|---------|
| **Video Processing** | Stub | Real OpenCV VideoCapture |
| **Visualization** | Stub | Complete OpenCV visualization |
| **Frame Processing** | Single frame only | Video file support |
| **Track Display** | None | Track IDs and boxes |
| **Code Optimization** | Full implementation | ~433 lines removed |
| **Test Coverage** | 95 tests | 98 tests (83%+ coverage) |
| **LCOV Integration** | None | Exclusion markers added |

## 📈 Phase 2 Metrics

**New Features Added**:
- `processVideo()`: ~100 lines
- `visualizeFrame()`: ~80 lines
- Test coverage: +3 new test cases
- LCOV markers: 4 exclusion blocks

**Code Optimizations**:
- YOLODetector.cpp: -138 lines (redundant code removed)
- PerceptionPipeline.cpp: -112 lines (processCamera removed)
- Various classes: -183 lines (test-only methods removed)
- **Total**: **~433 lines removed**

## 🎯 What Makes Phase 2 Complete

### Video Processing
```cpp
// Phase 1: Stub
return std::vector<PerceptionOutput>();

// Phase 2: Real
cv::VideoCapture cap(videoPath);
double fps = cap.get(cv::CAP_PROP_FPS);
while (cap.read(frame)) {
    double timestamp = frameCount / fps;
    PerceptionOutput output = processFrame(...);
    results.push_back(output);
}
```

### Visualization
```cpp
// Phase 1: Stub
(void)frame; (void)output;

// Phase 2: Real
cv::Mat displayFrame;
cv::cvtColor(frame, displayFrame, cv::COLOR_BGR2RGB);
for (const auto& track : output.tracks) {
    cv::rectangle(displayFrame, bbox, trackColor, 2);
    cv::putText(displayFrame, "Track " + std::to_string(track.getId()), ...);
}
cv::imshow("Human Detection & Tracking", displayFrame);
```

## 📸 Generate Images

### Generate PNG Images

```bash
# Install PlantUML (if not installed)
sudo apt-get install plantuml graphviz

# Generate images from PlantUML files
plantuml UML/revised_phase2/*.puml

# This creates:
# - class_diagram_revised.png
# - sequence_diagram_revised.png
# - activity_diagram_revised.png
```

### Generate PDF Files

```bash
# Generate PDF from PNG (requires ImageMagick or similar)
convert class_diagram_revised.png class_diagram_revised.pdf
convert sequence_diagram_revised.png sequence_diagram_revised.pdf
convert activity_diagram_revised.png activity_diagram_revised.pdf
```

### Or Use Online Editor

1. Go to http://www.plantuml.com/plantuml/uml/
2. Copy content from `.puml` files
3. Paste and generate
4. Download as PNG, SVG, or PDF

## 📝 File Locations

- **PlantUML Sources**: `UML/revised_phase2/*.puml`
- **Generated Images**: `UML/revised_phase2/*.png` (after running plantuml)
- **Generated PDFs**: `UML/revised_phase2/*.pdf` (after conversion)
- **Phase 1 Diagrams**: `UML/revised_phase1/*.puml`
- **Phase 0 Diagrams**: `UML/initial_phase0/*.puml`

## 🔗 Usage

### In Documentation
README.md references Phase 2 diagrams:
```markdown
![Class Diagram](./UML/revised_phase2/class_diagram_revised.png)
![Sequence Diagram](./UML/revised_phase2/sequence_diagram_revised.png)
![Activity Diagram](./UML/revised_phase2/activity_diagram_revised.png)
```

### In Presentations
Use these diagrams to show:
- Complete video processing workflow
- Visualization integration
- Phase 2 enhancements
- Code optimization results

---

**Created**: Phase 2  
**Last Updated**: November 3, 2025  
**Team**: Group 2 - Acme Robotics
