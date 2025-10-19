/**
 * @file perception_test.cpp
 * @brief Unit tests for perception system
 * @author Acme Robotics
 */

#include <gtest/gtest.h>
#include <memory>

#include "perception/detection/BoundingBox.hpp"
#include "perception/detection/Detection.hpp"
#include "perception/detection/YOLODetector.hpp"
#include "perception/tracking/Track.hpp"
#include "perception/tracking/KalmanFilter.hpp"
#include "perception/tracking/KalmanTracker.hpp"
#include "perception/core/CameraModel.hpp"
#include "perception/core/CoordinateTransformer.hpp"
#include "perception/core/PerceptionPipeline.hpp"
#include "perception/utils/Position3D.hpp"
#include "perception/utils/GeometryUtils.hpp"

using namespace perception;

// ============================================================================
// BoundingBox Tests
// ============================================================================

TEST(BoundingBoxTest, DefaultConstructor) {
  detection::BoundingBox bbox;
  EXPECT_FLOAT_EQ(bbox.getX(), 0.0f);
  EXPECT_FLOAT_EQ(bbox.getY(), 0.0f);
  EXPECT_FLOAT_EQ(bbox.getWidth(), 0.0f);
  EXPECT_FLOAT_EQ(bbox.getHeight(), 0.0f);
}

TEST(BoundingBoxTest, ParameterizedConstructor) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  EXPECT_FLOAT_EQ(bbox.getX(), 100.0f);
  EXPECT_FLOAT_EQ(bbox.getY(), 200.0f);
  EXPECT_FLOAT_EQ(bbox.getWidth(), 50.0f);
  EXPECT_FLOAT_EQ(bbox.getHeight(), 100.0f);
}

TEST(BoundingBoxTest, EdgeCalculations) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  EXPECT_FLOAT_EQ(bbox.getLeft(), 75.0f);
  EXPECT_FLOAT_EQ(bbox.getRight(), 125.0f);
  EXPECT_FLOAT_EQ(bbox.getTop(), 150.0f);
  EXPECT_FLOAT_EQ(bbox.getBottom(), 250.0f);
}

TEST(BoundingBoxTest, AreaCalculation) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  EXPECT_FLOAT_EQ(bbox.getArea(), 5000.0f);
}

TEST(BoundingBoxTest, IoUCalculation) {
  detection::BoundingBox bbox1(100.0f, 100.0f, 50.0f, 50.0f);
  detection::BoundingBox bbox2(100.0f, 100.0f, 50.0f, 50.0f);
  EXPECT_FLOAT_EQ(bbox1.computeIoU(bbox2), 1.0f);  // Perfect overlap

  detection::BoundingBox bbox3(200.0f, 200.0f, 50.0f, 50.0f);
  EXPECT_FLOAT_EQ(bbox1.computeIoU(bbox3), 0.0f);  // No overlap
}

// ============================================================================
// Detection Tests
// ============================================================================

TEST(DetectionTest, DefaultConstructor) {
  detection::Detection det;
  EXPECT_FLOAT_EQ(det.getConfidence(), 0.0f);
  EXPECT_EQ(det.getClassId(), -1);
  EXPECT_EQ(det.getClassName(), "unknown");
}

TEST(DetectionTest, ParameterizedConstructor) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  detection::Detection det(bbox, 0.85f, 0, "person");
  
  EXPECT_FLOAT_EQ(det.getConfidence(), 0.85f);
  EXPECT_EQ(det.getClassId(), 0);
  EXPECT_EQ(det.getClassName(), "person");
}

TEST(DetectionTest, ValidityCheck) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  detection::Detection det(bbox, 0.85f, 0, "person");
  
  EXPECT_TRUE(det.isValid(0.5f));
  EXPECT_FALSE(det.isValid(0.9f));
}

// ============================================================================
// YOLODetector Tests
// ============================================================================

TEST(YOLODetectorTest, Initialization) {
  detection::YOLODetector detector("models/yolov8n.onnx");
  EXPECT_TRUE(detector.isInitialized());
  EXPECT_FLOAT_EQ(detector.getConfidenceThreshold(), 0.5f);
  EXPECT_EQ(detector.getInputSize(), 640);
}

TEST(YOLODetectorTest, MockDetection) {
  detection::YOLODetector detector("models/yolov8n.onnx");
  
  // Create dummy frame
  int width = 640, height = 480;
  std::vector<unsigned char> frame(width * height * 3, 128);
  
  auto detections = detector.detect(frame.data(), width, height, 3);
  
  // Phase 0: Should return mock detections
  EXPECT_GT(detections.size(), 0);
}

TEST(YOLODetectorTest, ThresholdSetting) {
  detection::YOLODetector detector("models/yolov8n.onnx");
  detector.setConfidenceThreshold(0.7f);
  EXPECT_FLOAT_EQ(detector.getConfidenceThreshold(), 0.7f);
}

// ============================================================================
// Position3D Tests
// ============================================================================

TEST(Position3DTest, DefaultConstructor) {
  utils::Position3D pos;
  EXPECT_FLOAT_EQ(pos.getX(), 0.0f);
  EXPECT_FLOAT_EQ(pos.getY(), 0.0f);
  EXPECT_FLOAT_EQ(pos.getZ(), 0.0f);
}

TEST(Position3DTest, ParameterizedConstructor) {
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  EXPECT_FLOAT_EQ(pos.getX(), 1.0f);
  EXPECT_FLOAT_EQ(pos.getY(), 2.0f);
  EXPECT_FLOAT_EQ(pos.getZ(), 3.0f);
}

TEST(Position3DTest, VectorOperations) {
  utils::Position3D pos1(1.0f, 2.0f, 3.0f);
  utils::Position3D pos2(4.0f, 5.0f, 6.0f);
  
  utils::Position3D sum = pos1 + pos2;
  EXPECT_FLOAT_EQ(sum.getX(), 5.0f);
  EXPECT_FLOAT_EQ(sum.getY(), 7.0f);
  EXPECT_FLOAT_EQ(sum.getZ(), 9.0f);
  
  utils::Position3D diff = pos2 - pos1;
  EXPECT_FLOAT_EQ(diff.getX(), 3.0f);
  EXPECT_FLOAT_EQ(diff.getY(), 3.0f);
  EXPECT_FLOAT_EQ(diff.getZ(), 3.0f);
}

TEST(Position3DTest, ScalarOperations) {
  utils::Position3D pos(2.0f, 3.0f, 4.0f);
  
  utils::Position3D scaled = pos * 2.0f;
  EXPECT_FLOAT_EQ(scaled.getX(), 4.0f);
  EXPECT_FLOAT_EQ(scaled.getY(), 6.0f);
  EXPECT_FLOAT_EQ(scaled.getZ(), 8.0f);
}

TEST(Position3DTest, MagnitudeAndDistance) {
  utils::Position3D pos1(3.0f, 4.0f, 0.0f);
  EXPECT_FLOAT_EQ(pos1.magnitude(), 5.0f);
  
  utils::Position3D pos2(0.0f, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(pos1.distanceTo(pos2), 5.0f);
}

// ============================================================================
// Track Tests
// ============================================================================

TEST(TrackTest, Initialization) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  tracking::Track track(1, bbox, pos, 0.0);
  
  EXPECT_EQ(track.getId(), 1);
  EXPECT_EQ(track.getState(), tracking::TrackState::TENTATIVE);
  EXPECT_EQ(track.getHitCount(), 1);
  EXPECT_EQ(track.getMissCount(), 0);
}

TEST(TrackTest, Update) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  tracking::Track track(1, bbox, pos, 0.0);
  
  // Update track
  utils::Position3D newPos(1.5f, 2.5f, 3.5f);
  track.update(bbox, newPos, 0.1);
  
  EXPECT_EQ(track.getHitCount(), 2);
  EXPECT_EQ(track.getMissCount(), 0);
}

TEST(TrackTest, StateTransition) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  tracking::Track track(1, bbox, pos, 0.0);
  
  // After 3 updates, should become CONFIRMED
  for (int i = 0; i < 3; ++i) {
    track.update(bbox, pos, i * 0.1);
  }
  EXPECT_EQ(track.getState(), tracking::TrackState::CONFIRMED);
}

// ============================================================================
// KalmanFilter Tests
// ============================================================================

TEST(KalmanFilterTest, Initialization) {
  tracking::KalmanFilter kf(6, 3);
  EXPECT_FALSE(kf.isInitialized());
  
  std::vector<float> state = {0, 0, 0, 0, 0, 0};
  std::vector<float> cov(36, 0);
  for (int i = 0; i < 6; ++i) {
    cov[i * 6 + i] = 1.0f;
  }
  
  kf.initialize(state, cov);
  EXPECT_TRUE(kf.isInitialized());
}

TEST(KalmanFilterTest, PredictUpdate) {
  tracking::KalmanFilter kf(6, 3);
  
  std::vector<float> state = {1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f};
  std::vector<float> cov(36, 0);
  for (int i = 0; i < 6; ++i) {
    cov[i * 6 + i] = 1.0f;
  }
  
  kf.initialize(state, cov);
  kf.predict(0.1f);
  
  std::vector<float> measurement = {1.1f, 2.1f, 3.1f};
  kf.update(measurement);
  
  auto updatedState = kf.getState();
  EXPECT_EQ(updatedState.size(), 6);
}

// ============================================================================
// KalmanTracker Tests
// ============================================================================

TEST(KalmanTrackerTest, Initialization) {
  tracking::KalmanTracker tracker;
  EXPECT_EQ(tracker.getTrackCount(), 0);
}

TEST(KalmanTrackerTest, TrackCreationAndUpdate) {
  tracking::KalmanTracker tracker;
  
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  detection::Detection det(bbox, 0.85f, 0, "person");
  std::vector<detection::Detection> detections = {det};
  
  tracker.update(detections, 0.0);
  
  // Should create one new track (tentative)
  // After min_hits it becomes confirmed
  for (int i = 1; i < 5; ++i) {
    tracker.update(detections, i * 0.1);
  }
  
  EXPECT_GT(tracker.getTrackCount(), 0);
}

// ============================================================================
// CameraModel Tests
// ============================================================================

TEST(CameraModelTest, DefaultConstructor) {
  core::CameraModel camera;
  EXPECT_TRUE(camera.isCalibrated());
  EXPECT_GT(camera.getFocalLengthX(), 0.0f);
  EXPECT_GT(camera.getFocalLengthY(), 0.0f);
}

TEST(CameraModelTest, ParameterizedConstructor) {
  core::CameraModel camera(800.0f, 800.0f, 320.0f, 240.0f, 640, 480);
  EXPECT_FLOAT_EQ(camera.getFocalLengthX(), 800.0f);
  EXPECT_FLOAT_EQ(camera.getFocalLengthY(), 800.0f);
  EXPECT_EQ(camera.getImageWidth(), 640);
  EXPECT_EQ(camera.getImageHeight(), 480);
}

// ============================================================================
// CoordinateTransformer Tests
// ============================================================================

TEST(CoordinateTransformerTest, Initialization) {
  core::CameraModel camera;
  core::CoordinateTransformer transformer(camera);
  
  EXPECT_FLOAT_EQ(transformer.getAverageHumanHeight(), 1.7f);
}

TEST(CoordinateTransformerTest, ImageToRobotFrame) {
  core::CameraModel camera;
  core::CoordinateTransformer transformer(camera);
  
  detection::BoundingBox bbox(320.0f, 240.0f, 100.0f, 300.0f);
  utils::Position3D robotPos = transformer.imageToRobotFrame(bbox, 640, 480);
  
  // Should return valid position
  EXPECT_TRUE(std::isfinite(robotPos.getX()));
  EXPECT_TRUE(std::isfinite(robotPos.getY()));
  EXPECT_TRUE(std::isfinite(robotPos.getZ()));
}

// ============================================================================
// PerceptionPipeline Tests
// ============================================================================

TEST(PerceptionPipelineTest, Initialization) {
  auto detector = std::make_shared<detection::YOLODetector>(
      "models/yolov8n.onnx");
  auto tracker = std::make_shared<tracking::KalmanTracker>();
  core::CameraModel camera;
  auto transformer = std::make_shared<core::CoordinateTransformer>(camera);
  
  core::PerceptionPipeline pipeline(detector, tracker, transformer, camera);
  
  EXPECT_EQ(pipeline.getFrameCount(), 0);
  EXPECT_FALSE(pipeline.isVisualizationEnabled());
}

TEST(PerceptionPipelineTest, FrameProcessing) {
  auto detector = std::make_shared<detection::YOLODetector>(
      "models/yolov8n.onnx");
  auto tracker = std::make_shared<tracking::KalmanTracker>();
  core::CameraModel camera;
  auto transformer = std::make_shared<core::CoordinateTransformer>(camera);
  
  core::PerceptionPipeline pipeline(detector, tracker, transformer, camera);
  
  // Create dummy frame
  int width = 640, height = 480;
  std::vector<unsigned char> frame(width * height * 3, 128);
  
  core::PerceptionOutput output = pipeline.processFrame(
      frame.data(), width, height, 3, 0.0);
  
  EXPECT_TRUE(output.success);
  EXPECT_EQ(pipeline.getFrameCount(), 1);
}

// ============================================================================
// GeometryUtils Tests
// ============================================================================

TEST(GeometryUtilsTest, DepthEstimation) {
  detection::BoundingBox bbox(320.0f, 240.0f, 100.0f, 300.0f);
  float depth = utils::GeometryUtils::estimateDepthFromBboxHeight(
      bbox, 800.0f, 1.7f);
  
  EXPECT_GT(depth, 0.0f);
  EXPECT_TRUE(std::isfinite(depth));
}

TEST(GeometryUtilsTest, IntersectionArea) {
  detection::BoundingBox bbox1(100.0f, 100.0f, 50.0f, 50.0f);
  detection::BoundingBox bbox2(100.0f, 100.0f, 50.0f, 50.0f);
  
  float area = utils::GeometryUtils::computeIntersectionArea(bbox1, bbox2);
  EXPECT_FLOAT_EQ(area, 2500.0f);  // 50 * 50
}

TEST(GeometryUtilsTest, Clamp) {
  EXPECT_FLOAT_EQ(utils::GeometryUtils::clamp(5.0f, 0.0f, 10.0f), 5.0f);
  EXPECT_FLOAT_EQ(utils::GeometryUtils::clamp(-1.0f, 0.0f, 10.0f), 0.0f);
  EXPECT_FLOAT_EQ(utils::GeometryUtils::clamp(15.0f, 0.0f, 10.0f), 10.0f);
}

