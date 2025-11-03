/**
 * @file webcam_demo.cpp
 * @brief Webcam demonstration of Human Detection and Tracking
 * @author Acme Robotics
 * 
 * Real-time human detection and tracking using webcam input with visualization
 */

#include <iostream>
#include <memory>
#include <iomanip>

#include "perception/detection/YOLODetector.hpp"
#include "perception/tracking/KalmanTracker.hpp"
#include "perception/core/CameraModel.hpp"
#include "perception/core/CoordinateTransformer.hpp"
#include "perception/core/PerceptionPipeline.hpp"

using namespace perception;

int main(int argc, char* argv[]) {
  std::cout << "================================================" << std::endl;
  std::cout << "   Human Detection & Tracking - Webcam Demo    " << std::endl;
  std::cout << "================================================\n" << std::endl;

  // Parse command line arguments
  int cameraId = 0;
  std::string modelPath = "models/yolov5n.onnx";  // Default to YOLOv5 ONNX format (better OpenCV 4.5.4 compatibility)
  float confidenceThreshold = 0.5f;
  float nmsThreshold = 0.4f;
  int maxFrames = 0;  // 0 = unlimited

  if (argc > 1) {
    std::string arg1 = argv[1];
    if (arg1 == "--help" || arg1 == "-h") {
      std::cout << "Usage: webcam-demo [camera_id] [model_path] [confidence] [nms] [max_frames]\n" << std::endl;
      std::cout << "Arguments:" << std::endl;
      std::cout << "  camera_id      : Camera device ID (default: 0)" << std::endl;
      std::cout << "  model_path     : Path to YOLO model (default: models/yolov5n.onnx)" << std::endl;
      std::cout << "  confidence     : Confidence threshold 0.0-1.0 (default: 0.5)" << std::endl;
      std::cout << "  nms            : NMS threshold 0.0-1.0 (default: 0.4)" << std::endl;
      std::cout << "  max_frames     : Maximum frames to process, 0=unlimited (default: 0)" << std::endl;
      std::cout << "\nControls:" << std::endl;
      std::cout << "  Press 'q' or ESC to quit" << std::endl;
      return 0;
    }
    try {
      cameraId = std::stoi(arg1);
    } catch (...) {
      std::cerr << "Error: Invalid camera ID: " << arg1 << std::endl;
      return 1;
    }
  }
  if (argc > 2) {
    modelPath = argv[2];
  }
  if (argc > 3) {
    try {
      confidenceThreshold = std::stof(argv[3]);
    } catch (...) {
      std::cerr << "Error: Invalid confidence threshold: " << argv[3] << std::endl;
      return 1;
    }
  }
  if (argc > 4) {
    try {
      nmsThreshold = std::stof(argv[4]);
    } catch (...) {
      std::cerr << "Error: Invalid NMS threshold: " << argv[4] << std::endl;
      return 1;
    }
  }
  if (argc > 5) {
    try {
      maxFrames = std::stoi(argv[5]);
    } catch (...) {
      std::cerr << "Error: Invalid max frames: " << argv[5] << std::endl;
      return 1;
    }
  }

  std::cout << "Configuration:" << std::endl;
  std::cout << "  Camera ID: " << cameraId << std::endl;
  std::cout << "  Model: " << modelPath << std::endl;
  std::cout << "  Confidence Threshold: " << confidenceThreshold << std::endl;
  std::cout << "  NMS Threshold: " << nmsThreshold << std::endl;
  if (maxFrames > 0) {
    std::cout << "  Max Frames: " << maxFrames << std::endl;
  } else {
    std::cout << "  Max Frames: Unlimited" << std::endl;
  }
  std::cout << std::endl;

  try {
    // Initialize camera model
    std::cout << "1. Initializing camera model..." << std::endl;
    core::CameraModel camera(800.0f, 800.0f, 320.0f, 240.0f, 640, 480);
    
    // Set camera pose relative to robot base (0.5m above, forward facing)
    utils::Position3D cameraPos(0.0f, 0.0f, 0.5f);
    float rotation[9] = {1, 0, 0,  // Identity rotation
                        0, 1, 0,
                        0, 0, 1};
    camera.setCameraPose(cameraPos, rotation);
    std::cout << "   ✓ Camera parameters set successfully" << std::endl;

    // Initialize detector
    std::cout << "\n2. Initializing YOLO detector..." << std::endl;
    auto detector = std::make_shared<detection::YOLODetector>(
        modelPath, confidenceThreshold, nmsThreshold, 640);
    
    if (!detector->isInitialized()) {
      std::cerr << "   ✗ Warning: Detector initialization may have failed" << std::endl;
      std::cerr << "   Continuing with mock detection mode..." << std::endl;
    } else {
      std::cout << "   ✓ Detector initialized successfully" << std::endl;
    }

    // Initialize tracker
    std::cout << "\n3. Initializing Kalman tracker..." << std::endl;
    auto tracker = std::make_shared<tracking::KalmanTracker>(30, 3, 0.3f);
    std::cout << "   ✓ Tracker initialized" << std::endl;

    // Initialize coordinate transformer
    std::cout << "\n4. Initializing coordinate transformer..." << std::endl;
    auto transformer = std::make_shared<core::CoordinateTransformer>(camera);
    transformer->setAverageHumanHeight(1.7f);
    std::cout << "   ✓ Transformer initialized" << std::endl;

    // Create perception pipeline
    std::cout << "\n5. Creating perception pipeline..." << std::endl;
    core::PerceptionPipeline pipeline(detector, tracker, transformer, camera);
    std::cout << "   ✓ Pipeline created" << std::endl;

    // Process camera stream
    std::cout << "\n6. Starting webcam stream..." << std::endl;
    std::cout << "================================================\n" << std::endl;

    auto results = pipeline.processCamera(cameraId, maxFrames);

    // Print summary
    std::cout << "\n================================================" << std::endl;
    std::cout << "  Processing Complete!" << std::endl;
    std::cout << "  Total frames processed: " << results.size() << std::endl;
    
    if (!results.empty()) {
      int totalDetections = 0;
      int totalTracks = 0;
      for (const auto& result : results) {
        totalDetections += result.detections.size();
        totalTracks += result.tracks.size();
      }
      
      double avgDetections = static_cast<double>(totalDetections) / results.size();
      double avgTracks = static_cast<double>(totalTracks) / results.size();
      
      std::cout << "  Average detections per frame: " 
                << std::fixed << std::setprecision(2) << avgDetections << std::endl;
      std::cout << "  Average tracks per frame: " 
                << std::fixed << std::setprecision(2) << avgTracks << std::endl;
    }
    std::cout << "================================================\n" << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "\nError: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

