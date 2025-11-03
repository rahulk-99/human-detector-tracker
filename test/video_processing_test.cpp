/**
 * @file video_processing_test.cpp
 * @brief Unit tests for video processing functionality
 * @author Acme Robotics
 */

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "perception/core/PerceptionPipeline.hpp"
#include "perception/detection/YOLODetector.hpp"
#include "perception/tracking/KalmanTracker.hpp"
#include "perception/core/CameraModel.hpp"
#include "perception/core/CoordinateTransformer.hpp"

using namespace perception;

// Helper function to check if file exists
bool fileExists(const std::string& path) {
  std::ifstream file(path);
  return file.good();
}

// Helper function to find test data file
// Checks both relative to current directory and relative to project root
std::string findTestDataFile(const std::string& relativePath) {
  // Try current directory first (for when running from project root)
  if (fileExists(relativePath)) {
    return relativePath;
  }
  
  // Try from build directory - go up one level
  std::string fromBuild = "../" + relativePath;
  if (fileExists(fromBuild)) {
    return fromBuild;
  }
  
  // Try from build/test directory - go up two levels
  std::string fromBuildTest = "../../" + relativePath;
  if (fileExists(fromBuildTest)) {
    return fromBuildTest;
  }
  
  return relativePath; // Return original if not found
}

// ============================================================================
// Video Processing Tests
// ============================================================================

class VideoProcessingTest : public ::testing::Test {
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
  }

  std::shared_ptr<detection::YOLODetector> detector_;
  std::shared_ptr<tracking::KalmanTracker> tracker_;
  std::shared_ptr<core::CameraModel> camera_;
  std::shared_ptr<core::CoordinateTransformer> transformer_;
  std::unique_ptr<core::PerceptionPipeline> pipeline_;
};

// Test valid video path processing
TEST_F(VideoProcessingTest, ProcessValidVideoPath) {
  // Use the actual video file from the data directory
  std::string videoPath = findTestDataFile("data/ADL-Rundle-6-raw.mp4");
  
  // Check if file exists
  if (!fileExists(videoPath)) {
    GTEST_SKIP() << "Video file not found: " << videoPath;
  }
  
  std::vector<core::PerceptionOutput> results = pipeline_->processVideo(videoPath);
  
  // Should process at least some frames
  EXPECT_GT(results.size(), 0) << "Should process at least one frame from valid video";
  
  // Verify all results have valid structure
  for (const auto& result : results) {
    EXPECT_TRUE(result.success) << "All frames should process successfully";
    EXPECT_GE(result.timestamp, 0.0) << "Timestamp should be non-negative";
    EXPECT_GE(result.frameCount, 0) << "Frame count should be non-negative";
  }
}

// Test error handling for invalid video path
TEST_F(VideoProcessingTest, ProcessInvalidVideoPath) {
  std::string invalidPath = "nonexistent_video_file.mp4";
  
  std::vector<core::PerceptionOutput> results = pipeline_->processVideo(invalidPath);
  
  // Should return empty results for invalid path
  EXPECT_EQ(results.size(), 0) << "Invalid video path should return empty results";
}

// Test error handling for empty video path
TEST_F(VideoProcessingTest, ProcessEmptyVideoPath) {
  std::string emptyPath = "";
  
  std::vector<core::PerceptionOutput> results = pipeline_->processVideo(emptyPath);
  
  // Should handle empty path gracefully
  EXPECT_EQ(results.size(), 0) << "Empty video path should return empty results";
}

// Test timestamp calculation
TEST_F(VideoProcessingTest, TimestampCalculation) {
  std::string videoPath = findTestDataFile("data/ADL-Rundle-6-raw.mp4");
  
  if (!fileExists(videoPath)) {
    GTEST_SKIP() << "Video file not found: " << videoPath;
  }
  
  std::vector<core::PerceptionOutput> results = pipeline_->processVideo(videoPath);
  
  if (results.size() > 1) {
    // Timestamps should be monotonically increasing
    for (size_t i = 1; i < results.size(); ++i) {
      EXPECT_GE(results[i].timestamp, results[i-1].timestamp) 
          << "Timestamps should be monotonically increasing";
    }
    
    // First frame should have timestamp >= 0
    EXPECT_GE(results[0].timestamp, 0.0) << "First frame timestamp should be >= 0";
  }
}

// Test frame count increment
TEST_F(VideoProcessingTest, FrameCountIncrement) {
  std::string videoPath = findTestDataFile("data/ADL-Rundle-6-raw.mp4");
  
  if (!fileExists(videoPath)) {
    GTEST_SKIP() << "Video file not found: " << videoPath;
  }
  
  std::vector<core::PerceptionOutput> results = pipeline_->processVideo(videoPath);
  
  if (results.size() > 1) {
    // Frame counts should be sequential
    for (size_t i = 0; i < results.size(); ++i) {
      EXPECT_EQ(results[i].frameCount, static_cast<int>(i + 1)) 
          << "Frame count should increment sequentially";
    }
  }
}

// Test results collection
TEST_F(VideoProcessingTest, ResultsCollection) {
  std::string videoPath = findTestDataFile("data/ADL-Rundle-6-raw.mp4");
  
  if (!fileExists(videoPath)) {
    GTEST_SKIP() << "Video file not found: " << videoPath;
  }
  
  std::vector<core::PerceptionOutput> results = pipeline_->processVideo(videoPath);
  
  // Each result should have valid structure
  for (size_t i = 0; i < results.size(); ++i) {
    const auto& result = results[i];
    
    // Verify structure members exist
    EXPECT_TRUE(result.success) << "Result should have success flag";
    EXPECT_GE(result.timestamp, 0.0) << "Result should have valid timestamp";
    EXPECT_EQ(result.frameCount, static_cast<int>(i + 1)) << "Result should have correct frame count";
    
    // Detections and tracks are vectors (can be empty)
    EXPECT_GE(result.detections.size(), 0) << "Detections should be a valid vector";
    EXPECT_GE(result.tracks.size(), 0) << "Tracks should be a valid vector";
  }
}

// Integration test with real video file
TEST_F(VideoProcessingTest, IntegrationTestWithRealVideo) {
  std::string videoPath = findTestDataFile("data/ADL-Rundle-6-raw.mp4");
  
  if (!fileExists(videoPath)) {
    GTEST_SKIP() << "Video file not found: " << videoPath;
  }
  
  // Process video
  std::vector<core::PerceptionOutput> results = pipeline_->processVideo(videoPath);
  
  // Should process multiple frames
  EXPECT_GT(results.size(), 10) << "Should process multiple frames from video";
  
  // Verify video cleanup (no crashes after processing)
  // This is implicit - if we get here without crashing, cleanup worked
  
  // Check that pipeline is still functional after processing
  EXPECT_GE(pipeline_->getFrameCount(), static_cast<int>(results.size())) 
      << "Pipeline frame count should match processed frames";
}

