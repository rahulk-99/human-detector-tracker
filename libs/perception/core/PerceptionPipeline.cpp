/**
 * @file PerceptionPipeline.cpp
 * @brief Implementation of PerceptionPipeline class
 * @author Acme Robotics
 * 
 * Phase 0: Stub implementation without OpenCV visualization
 */

#include "perception/core/PerceptionPipeline.hpp"
#include <iostream>

namespace perception {
namespace core {

PerceptionPipeline::PerceptionPipeline(
    std::shared_ptr<detection::IDetector> detector,
    std::shared_ptr<tracking::ITracker> tracker,
    std::shared_ptr<ICoordinateTransform> transformer,
    const CameraModel& cameraModel)
    : detector_(detector),
      tracker_(tracker),
      transformer_(transformer),
      cameraModel_(cameraModel),
      frameCount_(0),
      visualizationEnabled_(false) {
  std::cout << "[PerceptionPipeline] Initialized" << std::endl;
}

PerceptionOutput PerceptionPipeline::processFrame(const unsigned char* frame,
                                                 int width,
                                                 int height,
                                                 int channels,
                                                 double timestamp) {
  PerceptionOutput output;
  output.timestamp = timestamp;
  output.frameCount = ++frameCount_;
  output.success = false;

  try {
    // Step 1: Detect humans
    std::vector<detection::Detection> detections =
        detector_->detect(frame, width, height, channels);

    std::cout << "[Frame " << frameCount_ << "] Detected " << detections.size()
              << " humans" << std::endl;

    // Step 2: Transform detections to 3D positions in robot frame
    for (auto& det : detections) {
      // Get 3D position for this detection
      utils::Position3D pos3D = transformer_->imageToRobotFrame(
          det.getBoundingBox(), width, height);
      
      std::cout << "  Detection at: (" << pos3D.getX() << ", " << pos3D.getY()
                << ", " << pos3D.getZ() << ") m" << std::endl;
    }

    // Step 3: Update tracker with detections
    tracker_->update(detections, timestamp);

    // Step 4: Get active tracks
    output.tracks = tracker_->getActiveTracks();
    std::cout << "  Active tracks: " << output.tracks.size() << std::endl;

    for (const auto& track : output.tracks) {
      const auto& pos = track.getPosition();
      std::cout << "    Track " << track.getId() << ": (" << pos.getX()
                << ", " << pos.getY() << ", " << pos.getZ() << ") m"
                << std::endl;
    }

    output.success = true;

    // Visualize if enabled (Phase 0: stub)
    if (visualizationEnabled_) {
      visualizeFrame(frame, width, height, output);
    }

  } catch (const std::exception& e) {
    std::cerr << "[PerceptionPipeline] Error processing frame: " << e.what()
              << std::endl;
  }

  return output;
}

std::vector<PerceptionOutput> PerceptionPipeline::processVideo(
    const std::string& videoPath) {
  std::vector<PerceptionOutput> results;
  
  std::cout << "[PerceptionPipeline] Processing video: " << videoPath
            << std::endl;
  std::cout << "[PerceptionPipeline] Phase 0 - Stub implementation"
            << std::endl;

  // Phase 0: Return empty results
  return results;
}

std::vector<PerceptionOutput> PerceptionPipeline::processCamera(
    int cameraId,
    int maxFrames) {
  std::vector<PerceptionOutput> results;
  
  std::cout << "[PerceptionPipeline] Processing camera " << cameraId
            << std::endl;
  std::cout << "[PerceptionPipeline] Phase 0 - Stub implementation"
            << std::endl;

  // Phase 0: Return empty results
  return results;
}

void PerceptionPipeline::reset() {
  tracker_->reset();
  frameCount_ = 0;
  std::cout << "[PerceptionPipeline] Reset" << std::endl;
}

std::shared_ptr<detection::IDetector> PerceptionPipeline::getDetector() const {
  return detector_;
}

std::shared_ptr<tracking::ITracker> PerceptionPipeline::getTracker() const {
  return tracker_;
}

std::shared_ptr<ICoordinateTransform> PerceptionPipeline::getTransformer() const {
  return transformer_;
}

const CameraModel& PerceptionPipeline::getCameraModel() const {
  return cameraModel_;
}

int PerceptionPipeline::getFrameCount() const {
  return frameCount_;
}

void PerceptionPipeline::setVisualization(bool enable) {
  visualizationEnabled_ = enable;
}

bool PerceptionPipeline::isVisualizationEnabled() const {
  return visualizationEnabled_;
}

void PerceptionPipeline::visualizeFrame(const unsigned char* frame,
                                       int width,
                                       int height,
                                       const PerceptionOutput& output) {
  // Phase 0: Stub - no visualization
  std::cout << "[PerceptionPipeline] Visualization (stub)" << std::endl;
}

}  // namespace core
}  // namespace perception

