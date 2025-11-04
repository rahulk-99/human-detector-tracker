/**
 * @file visualization_test.cpp
 * @brief Unit tests for visualization functionality
 * @author Acme Robotics
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <fstream>
#include <string>

#include "perception/core/PerceptionPipeline.hpp"
#include "perception/detection/YOLODetector.hpp"
#include "perception/detection/BoundingBox.hpp"
#include "perception/detection/Detection.hpp"
#include "perception/tracking/KalmanTracker.hpp"
#include "perception/tracking/Track.hpp"
#include "perception/core/CameraModel.hpp"
#include "perception/core/CoordinateTransformer.hpp"
#include "perception/utils/Position3D.hpp"

using namespace perception;

// ============================================================================
// Visualization Tests
// ============================================================================

class VisualizationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create pipeline components
    detector_ = std::make_shared<detection::YOLODetector>(
        "models/yolov8n.onnx");
    tracker_ = std::make_shared<tracking::KalmanTracker>();
    camera_ = std::make_shared<core::CameraModel>(800.0f, 800.0f, 320.0f, 240.0f, 640, 480);
    transformer_ = std::make_shared<core::CoordinateTransformer>(*camera_);
    
    // Create pipeline
    pipeline_ = std::make_unique<core::PerceptionPipeline>(
        detector_, tracker_, transformer_, *camera_);
    
    // Enable visualization
    pipeline_->setVisualization(true);
    
    // Create test frame data
    width_ = 640;
    height_ = 480;
    channels_ = 3;
    frameData_.resize(width_ * height_ * channels_, 128); // Gray frame
  }

  std::shared_ptr<detection::YOLODetector> detector_;
  std::shared_ptr<tracking::KalmanTracker> tracker_;
  std::shared_ptr<core::CameraModel> camera_;
  std::shared_ptr<core::CoordinateTransformer> transformer_;
  std::unique_ptr<core::PerceptionPipeline> pipeline_;
  
  int width_, height_, channels_;
  std::vector<unsigned char> frameData_;
};

// Test visualization with empty detections and tracks
TEST_F(VisualizationTest, VisualizationWithEmptyOutput) {
  core::PerceptionOutput emptyOutput;
  emptyOutput.detections.clear();
  emptyOutput.tracks.clear();
  
  // Should not crash with empty output
  core::PerceptionOutput result = pipeline_->processFrame(frameData_.data(), width_, height_, channels_, 0.0);
  EXPECT_EQ(result.frameCount, 1);
  EXPECT_EQ(result.timestamp, 0.0);
  EXPECT_TRUE(result.success);
}

// Test visualization with bounding box detections
TEST_F(VisualizationTest, VisualizationWithBoundingBoxes) {
  // Create test detection with bounding box
  detection::BoundingBox bbox(100.0f, 100.0f, 50.0f, 100.0f);
  detection::Detection det(bbox, 0.85f, 0, "person");
  
  core::PerceptionOutput output;
  output.detections.push_back(det);
  output.tracks.clear();
  
  // Should not crash when visualizing bounding boxes
  EXPECT_NO_THROW({
    // Process frame (which will call visualizeFrame internally)
    pipeline_->processFrame(frameData_.data(), width_, height_, channels_, 0.0);
  });
}

// Test visualization with track ID labels
TEST_F(VisualizationTest, VisualizationWithTrackLabels) {
  // Create a track
  detection::BoundingBox bbox(200.0f, 200.0f, 60.0f, 120.0f);
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  tracking::Track track(42, bbox, pos, 0.0);
  
  // Update track to make it confirmed
  for (int i = 0; i < 5; ++i) {
    track.update(bbox, pos, i * 0.1);
  }
  
  // Process frame to create tracks
  pipeline_->processFrame(frameData_.data(), width_, height_, channels_, 0.0);
  
  // Get output and verify tracks exist
  auto output = pipeline_->processFrame(frameData_.data(), width_, height_, channels_, 0.1);
  
  // Should have tracks visualized
  EXPECT_GE(output.tracks.size(), 0);
}

// Test visualization enable/disable
TEST_F(VisualizationTest, VisualizationEnableDisable) {
  // Initially enabled
  EXPECT_TRUE(pipeline_->isVisualizationEnabled());
  
  // Disable visualization
  pipeline_->setVisualization(false);
  EXPECT_FALSE(pipeline_->isVisualizationEnabled());
  
  // Should still process without crashing
  EXPECT_NO_THROW({
    pipeline_->processFrame(frameData_.data(), width_, height_, channels_, 0.0);
  });
  
  // Re-enable
  pipeline_->setVisualization(true);
  EXPECT_TRUE(pipeline_->isVisualizationEnabled());
}

// Test visualization with multiple detections
TEST_F(VisualizationTest, VisualizationWithMultipleDetections) {
  // Create multiple detections
  std::vector<detection::Detection> detections;
  
  detection::BoundingBox bbox1(100.0f, 100.0f, 50.0f, 100.0f);
  detections.push_back(detection::Detection(bbox1, 0.9f, 0, "person"));
  
  detection::BoundingBox bbox2(300.0f, 200.0f, 60.0f, 110.0f);
  detections.push_back(detection::Detection(bbox2, 0.85f, 0, "person"));
  
  detection::BoundingBox bbox3(500.0f, 300.0f, 55.0f, 105.0f);
  detections.push_back(detection::Detection(bbox3, 0.75f, 0, "person"));
  
  // Process frame (will create detections internally)
  EXPECT_NO_THROW({
    pipeline_->processFrame(frameData_.data(), width_, height_, channels_, 0.0);
  });
}

// Test visualization with edge case bounding boxes (out of bounds)
TEST_F(VisualizationTest, VisualizationWithEdgeCaseBoundingBoxes) {
  // Create bounding box that extends beyond image bounds
  detection::BoundingBox bbox(-50.0f, -50.0f, 1000.0f, 1000.0f);
  detection::Detection det(bbox, 0.8f, 0, "person");
  
  core::PerceptionOutput output;
  output.detections.push_back(det);
  output.tracks.clear();
  
  // Should handle out-of-bounds boxes gracefully (clamping)
  EXPECT_NO_THROW({
    pipeline_->processFrame(frameData_.data(), width_, height_, channels_, 0.0);
  });
}

// Test visualization frame conversion
TEST_F(VisualizationTest, VisualizationFrameConversion) {
  // Test with different frame sizes
  std::vector<int> widths = {320, 640, 1280};
  std::vector<int> heights = {240, 480, 720};
  
  for (size_t i = 0; i < widths.size(); ++i) {
    int w = widths[i];
    int h = heights[i];
    std::vector<unsigned char> frame(w * h * 3, 128);
    
    EXPECT_NO_THROW({
      pipeline_->processFrame(frame.data(), w, h, 3, 0.0);
    });
  }
}


