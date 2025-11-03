/**
 * @file main.cpp
 * @brief Main application demonstrating Human Perception System
 * @author Acme Robotics
 */

#include <iostream>
#include <memory>
#include <vector>

#include "perception/detection/YOLODetector.hpp"
#include "perception/tracking/KalmanTracker.hpp"
#include "perception/core/CameraModel.hpp"
#include "perception/core/CoordinateTransformer.hpp"
#include "perception/core/PerceptionPipeline.hpp"

using namespace perception;

/**
 * @brief Print perception output to console
 */
void printOutput(const core::PerceptionOutput& output) {
  std::cout << "\n========================================" << std::endl;
  std::cout << "Frame #" << output.frameCount
            << " | Timestamp: " << output.timestamp << "s" << std::endl;
  std::cout << "Active Tracks: " << output.tracks.size() << std::endl;
  
  for (const auto& track : output.tracks) {
    const auto& pos = track.getPosition();
    const auto& vel = track.getVelocity();
    
    std::cout << "  Track ID " << track.getId() << ":" << std::endl;
    std::cout << "    Position (m): ("
              << pos.getX() << ", "
              << pos.getY() << ", "
              << pos.getZ() << ")" << std::endl;
    std::cout << "    Velocity (m/s): ("
              << vel.getX() << ", "
              << vel.getY() << ", "
              << vel.getZ() << ")" << std::endl;
    std::cout << "    Hit Count: " << track.getHitCount()
              << " | Age: " << track.getAge() << std::endl;
  }
  std::cout << "========================================\n" << std::endl;
}

int main(int argc, char* argv[]) {
  std::cout << "================================================" << std::endl;
  std::cout << "   Acme Robotics - Human Perception System     " << std::endl;
  std::cout << "   Phase 1: Detection & Tracking Demo          " << std::endl;
  std::cout << "================================================\n" << std::endl;

  // Parse command line arguments (video or webcam)
  bool useVideo = false;
  std::string videoPath;
  int cameraId = 0;
  std::string modelPath = "models/yolov8n.onnx";
  // Fixed thresholds (kept within the script)
  const float confidenceThreshold = 0.5f;
  const float nmsThreshold = 0.4f;
  int maxFrames = 0;  // webcam mode only

  int argi = 1;
  if (argc > argi) {
    std::string arg1 = argv[argi];
    if (arg1 == "--help" || arg1 == "-h") {
      std::cout << "Usage:\n"
                << "  shell-app --video <path> [model_path]\n"
                << "  shell-app [camera_id] [model_path] [max_frames]\n" << std::endl;
      return 0;
    }
    // Handle --video (single argument) or -- video (two arguments)
    if (arg1 == "--video" || (arg1 == "--" && argc > argi + 1 && std::string(argv[argi + 1]) == "video")) {
      if (arg1 == "--") {
        argi++;  // Skip the "--"
      }
      if (argc <= argi + 1) {
        std::cerr << "Error: --video requires a path" << std::endl;
        return 1;
      }
      useVideo = true;
      videoPath = argv[argi + 1];
      argi += 2;
    } else if (arg1 == "--") {
      // If just "--" is provided without "video", give helpful error
      std::cerr << "Error: Invalid argument '--'. Did you mean '--video'?" << std::endl;
      std::cerr << "Usage: shell-app --video <path> [model_path]" << std::endl;
      return 1;
    } else {
      try {
        cameraId = std::stoi(arg1);
        argi += 1;
      } catch (...) {
        useVideo = true;
        videoPath = arg1;
        argi += 1;
      }
    }
  }
  if (argc > argi) {
    modelPath = argv[argi++];
  }
  // confidenceThreshold and nmsThreshold are fixed in code
  if (!useVideo && argc > argi) {
    try { maxFrames = std::stoi(argv[argi++]); }
    catch (...) { std::cerr << "Invalid max_frames" << std::endl; return 1; }
  }

  std::cout << "Configuration:" << std::endl;
  if (useVideo) {
    std::cout << "  Mode: Video\n  Video: " << videoPath << std::endl;
  } else {
    std::cout << "  Mode: Webcam\n  Camera ID: " << cameraId << std::endl;
  }
  std::cout << "  Model: " << modelPath << std::endl;
  std::cout << "  Confidence: " << confidenceThreshold << std::endl;
  std::cout << "  NMS: " << nmsThreshold << std::endl;
  if (!useVideo) {
    std::cout << "  Max Frames: " << (maxFrames > 0 ? std::to_string(maxFrames) : std::string("Unlimited")) << std::endl;
  }
  std::cout << std::endl;

  // Initialize camera model
  std::cout << "1. Initializing camera model..." << std::endl;
  core::CameraModel camera(800.0f, 800.0f, 320.0f, 240.0f, 640, 480);
  
  // Set camera pose relative to robot base (0.5m above, forward facing)
  utils::Position3D cameraPos(0.0f, 0.0f, 0.5f);
  float rotation[9] = {1, 0, 0,  // Identity rotation
                      0, 1, 0,
                      0, 0, 1};
  camera.setCameraPose(cameraPos, rotation);
  std::cout << "   Camera parameters set successfully" << std::endl;

  // Initialize detector
  std::cout << "\n2. Initializing YOLO detector..." << std::endl;
  auto detector = std::make_shared<detection::YOLODetector>(
      modelPath, confidenceThreshold, nmsThreshold, 640);

  // Initialize tracker
  std::cout << "\n3. Initializing Kalman tracker..." << std::endl;
  auto tracker = std::make_shared<tracking::KalmanTracker>(30, 3, 0.3f);

  // Initialize coordinate transformer
  std::cout << "\n4. Initializing coordinate transformer..." << std::endl;
  auto transformer = std::make_shared<core::CoordinateTransformer>(camera);
  transformer->setAverageHumanHeight(1.7f);

  // Create perception pipeline
  std::cout << "\n5. Creating perception pipeline..." << std::endl;
  core::PerceptionPipeline pipeline(detector, tracker, transformer, camera);

  // Run pipeline on video or webcam
  std::cout << "\n6. Starting " << (useVideo ? "video" : "webcam") << " stream...\n" << std::endl;
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
    for (const auto& r : results) {
      totalDetections += static_cast<int>(r.detections.size());
      totalTracks += static_cast<int>(r.tracks.size());
    }
    double avgDet = static_cast<double>(totalDetections) / results.size();
    double avgTrk = static_cast<double>(totalTracks) / results.size();
    std::cout << "  Average detections per frame: " << avgDet << std::endl;
    std::cout << "  Average tracks per frame: " << avgTrk << std::endl;
  }
  std::cout << "================================================\n" << std::endl;

  std::cout << "\n================================================" << std::endl;
  std::cout << "  Demo Complete!" << std::endl;
  std::cout << "  Phase 1: Detection & Tracking verified" << std::endl;
  std::cout << "================================================\n" << std::endl;

  return 0;
}
