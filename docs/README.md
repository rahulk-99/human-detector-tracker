# Documentation Directory

This directory contains the generated Doxygen documentation for the Human Detector Tracker project.

## Generation

Run the below command to generate the docs:

```bash
cmake --build build/ --target docs
```

## Contents

### `html/` Directory
Complete HTML documentation including:
- **Class Documentation**: Detailed API documentation for all classes
- **Function Documentation**: Method signatures, parameters, and return values
- **Namespace Documentation**: Organized by perception modules
- **File Documentation**: Source code documentation
- **Graphs and Diagrams**: Call graphs, inheritance diagrams, and collaboration diagrams
- **Index Pages**: Alphabetical and hierarchical class listings

## Key Documentation Sections

### Core Modules
- **CameraModel**: Camera calibration and pose management
- **CoordinateTransformer**: 2D to 3D coordinate transformations
- **PerceptionPipeline**: Main processing pipeline

### Detection Module
- **YOLODetector**: YOLO-based human detection implementation
- **BoundingBox**: 2D bounding box operations and IoU calculations
- **Detection**: Detection result data structures

### Tracking Module
- **KalmanFilter**: Kalman filter implementation for state estimation
- **KalmanTracker**: Multi-object tracking using Kalman filters
- **Track**: Individual track management and state transitions

### Utility Modules
- **Position3D**: 3D position and vector operations
- **GeometryUtils**: Geometric calculations and transformations

## Usage

1. **Open `html/index.html`** in a web browser to view the complete documentation
2. **Navigate by class** using the alphabetical class index
3. **Explore inheritance** using the hierarchical class index
4. **View call graphs** for understanding function relationships
5. **Check file documentation** for source code details
