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

int main() {
  std::cout << "================================================" << std::endl;
  std::cout << "   Acme Robotics - Human Perception System     " << std::endl;
  std::cout << "   Phase 0: Design and Architecture Demo       " << std::endl;
  std::cout << "================================================\n" << std::endl;

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
      "models/yolov8n.onnx", 0.5f, 0.4f, 640);

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

  // Simulate processing multiple frames
  std::cout << "\n6. Processing frames...\n" << std::endl;
  
  int width = 640, height = 480;
  std::vector<unsigned char> frame(width * height * 3, 128);

  // Process 5 frames
  for (int i = 0; i < 5; ++i) {
    double timestamp = i * 0.033;  // ~30 FPS
    
    core::PerceptionOutput output = pipeline.processFrame(
        frame.data(), width, height, 3, timestamp);
    
    if (output.success) {
      printOutput(output);
    }
  }

  std::cout << "\n================================================" << std::endl;
  std::cout << "  Demo Complete!" << std::endl;
  std::cout << "  Phase 0: Architecture design verified" << std::endl;
  std::cout << "  Next: Implement full YOLO and Kalman algorithms" << std::endl;
  std::cout << "================================================\n" << std::endl;

  return 0;
}
