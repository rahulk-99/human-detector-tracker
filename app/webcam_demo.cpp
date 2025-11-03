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
  bool useVideo = false;
  std::string videoPath;
  int cameraId = 0;
  std::string modelPath = "models/yolov5n.onnx";  // Default model
  // Fixed thresholds (kept within the script)
  const float confidenceThreshold = 0.5f;
  const float nmsThreshold = 0.4f;
  int maxFrames = 0;  // 0 = unlimited

  int argi = 1;
  if (argc > argi) {
    std::string arg1 = argv[argi];
    if (arg1 == "--help" || arg1 == "-h") {
      std::cout << "Usage:\n"
                << "  webcam-demo [camera_id] [model_path] [max_frames]\n"
                << "  webcam-demo --video <path> [model_path]\n" << std::endl;
      std::cout << "Arguments:" << std::endl;
      std::cout << "  camera_id      : Camera device ID (default: 0)" << std::endl;
      std::cout << "  --video <path> : Run on a video file instead of webcam" << std::endl;
      std::cout << "  model_path     : Path to YOLO model (default: models/yolov5n.onnx)" << std::endl;
      // thresholds are fixed in code
      std::cout << "  max_frames     : For webcam mode only; 0=unlimited (default: 0)" << std::endl;
      std::cout << "\nControls:" << std::endl;
      std::cout << "  Press 'q' or ESC to quit" << std::endl;
      return 0;
    }
    if (arg1 == "--video") {
      if (argc <= argi + 1) {
        std::cerr << "Error: --video requires a path" << std::endl;
        return 1;
      }
      useVideo = true;
      videoPath = argv[argi + 1];
      argi += 2;
    } else {
      try {
        cameraId = std::stoi(arg1);
        argi += 1;
      } catch (...) {
        // If not an int, treat as video path
        useVideo = true;
        videoPath = arg1;
        argi += 1;
      }
    }
  }
  if (argc > argi) {
    modelPath = argv[argi++];
  }
  // Confidence and NMS thresholds are fixed in code
  if (!useVideo && argc > argi) {
    try {
      maxFrames = std::stoi(argv[argi++]);
    } catch (...) {
      std::cerr << "Error: Invalid max frames: " << argv[argi-1] << std::endl;
      return 1;
    }
  }

  std::cout << "Configuration:" << std::endl;
  if (useVideo) {
    std::cout << "  Mode: Video" << std::endl;
    std::cout << "  Video: " << videoPath << std::endl;
  } else {
    std::cout << "  Mode: Webcam" << std::endl;
    std::cout << "  Camera ID: " << cameraId << std::endl;
  }
  std::cout << "  Model: " << modelPath << std::endl;
  std::cout << "  Confidence Threshold: " << confidenceThreshold << std::endl;
  std::cout << "  NMS Threshold: " << nmsThreshold << std::endl;
  if (!useVideo && maxFrames > 0) {
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

    // Process stream
    std::cout << "\n6. Starting " << (useVideo ? "video" : "webcam") << " stream..." << std::endl;
    std::cout << "================================================\n" << std::endl;

    std::vector<core::PerceptionOutput> results;
    if (useVideo) {
      results = pipeline.processVideo(videoPath);
    } else {
      results = pipeline.processCamera(cameraId, maxFrames);
    }

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

