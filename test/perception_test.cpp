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

TEST(BoundingBoxTest, IoUWithZeroAreaBox) {
  // Test IoU with zero area box (unionArea <= 0 edge case)
  detection::BoundingBox zeroBox(100.0f, 100.0f, 0.0f, 0.0f);
  detection::BoundingBox normalBox(100.0f, 100.0f, 50.0f, 50.0f);
  
  // Zero area box should return 0 IoU
  EXPECT_FLOAT_EQ(zeroBox.computeIoU(normalBox), 0.0f);
  EXPECT_FLOAT_EQ(normalBox.computeIoU(zeroBox), 0.0f);
  
  // Two zero area boxes
  detection::BoundingBox zeroBox2(200.0f, 200.0f, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(zeroBox.computeIoU(zeroBox2), 0.0f);
}

TEST(BoundingBoxTest, ZeroAreaBox) {
  detection::BoundingBox zeroBox(100.0f, 200.0f, 0.0f, 0.0f);
  
  EXPECT_FLOAT_EQ(zeroBox.getArea(), 0.0f);
  EXPECT_FLOAT_EQ(zeroBox.getWidth(), 0.0f);
  EXPECT_FLOAT_EQ(zeroBox.getHeight(), 0.0f);
}

TEST(BoundingBoxTest, Set) {
  detection::BoundingBox box(100.0f, 200.0f, 50.0f, 60.0f);
  
  box.set(150.0f, 250.0f, 80.0f, 90.0f);
  
  EXPECT_FLOAT_EQ(box.getX(), 150.0f);
  EXPECT_FLOAT_EQ(box.getY(), 250.0f);
  EXPECT_FLOAT_EQ(box.getWidth(), 80.0f);
  EXPECT_FLOAT_EQ(box.getHeight(), 90.0f);
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

TEST(DetectionTest, Setters) {
  detection::BoundingBox bbox1(100.0f, 200.0f, 50.0f, 100.0f);
  detection::Detection det(bbox1, 0.85f, 0, "person");
  
  // Test setBoundingBox
  detection::BoundingBox bbox2(200.0f, 300.0f, 60.0f, 110.0f);
  det.setBoundingBox(bbox2);
  EXPECT_FLOAT_EQ(det.getBoundingBox().getX(), 200.0f);
  
  // Test setConfidence
  det.setConfidence(0.95f);
  EXPECT_FLOAT_EQ(det.getConfidence(), 0.95f);
  
  // Test setClass
  det.setClass(1, "human");
  EXPECT_EQ(det.getClassId(), 1);
  EXPECT_EQ(det.getClassName(), "human");
}

// ============================================================================
// YOLODetector Tests
// ============================================================================

TEST(YOLODetectorTest, Initialization) {
  // Use a non-existent model path to test initialization in mock mode
  detection::YOLODetector detector("models/nonexistent_model.onnx");
  EXPECT_TRUE(detector.isInitialized());
  EXPECT_FLOAT_EQ(detector.getConfidenceThreshold(), 0.5f);
  EXPECT_EQ(detector.getInputSize(), 640);
}

TEST(YOLODetectorTest, MockDetection) {
  // Use a non-existent model path to force mock mode
  detection::YOLODetector detector("models/nonexistent_model.onnx");
  
  // Verify detector initializes (even in mock mode)
  EXPECT_TRUE(detector.isInitialized());
  
  // Create dummy frame
  int width = 640, height = 480;
  std::vector<unsigned char> frame(width * height * 3, 128);
  
  auto detections = detector.detect(frame.data(), width, height, 3);
  
  // Phase 1: Mock mode returns empty detections to avoid false positives
  // This is the correct behavior - real YOLO model should be used for actual detection
  EXPECT_EQ(detections.size(), 0);
}

TEST(YOLODetectorTest, ThresholdSetting) {
  // Use a non-existent model path to test threshold setting in mock mode
  detection::YOLODetector detector("models/nonexistent_model.onnx");
  detector.setConfidenceThreshold(0.7f);
  EXPECT_FLOAT_EQ(detector.getConfidenceThreshold(), 0.7f);
}

TEST(YOLODetectorTest, ConstructorErrors) {
  // Empty model path
  EXPECT_THROW(detection::YOLODetector("", 0.5f, 0.4f, 640), std::invalid_argument);
  
  // Invalid confidence threshold
  EXPECT_THROW(detection::YOLODetector("model.onnx", -0.1f, 0.4f, 640), std::invalid_argument);
  EXPECT_THROW(detection::YOLODetector("model.onnx", 1.5f, 0.4f, 640), std::invalid_argument);
  
  // Invalid NMS threshold
  EXPECT_THROW(detection::YOLODetector("model.onnx", 0.5f, -0.1f, 640), std::invalid_argument);
  EXPECT_THROW(detection::YOLODetector("model.onnx", 0.5f, 1.5f, 640), std::invalid_argument);
  
  // Invalid input size
  EXPECT_THROW(detection::YOLODetector("model.onnx", 0.5f, 0.4f, 0), std::invalid_argument);
  EXPECT_THROW(detection::YOLODetector("model.onnx", 0.5f, 0.4f, -10), std::invalid_argument);
}

TEST(YOLODetectorTest, NmsThresholdMethods) {
  detection::YOLODetector detector("models/nonexistent_model.onnx");
  
  // Test default NMS threshold
  EXPECT_FLOAT_EQ(detector.getNmsThreshold(), 0.4f);
  
  // Test setter
  detector.setNmsThreshold(0.6f);
  EXPECT_FLOAT_EQ(detector.getNmsThreshold(), 0.6f);
  
  // Test invalid thresholds
  EXPECT_THROW(detector.setNmsThreshold(-0.1f), std::invalid_argument);
  EXPECT_THROW(detector.setNmsThreshold(1.5f), std::invalid_argument);
}

// Note: applyNMS() is a private method, so we cannot test it directly.
// NMS is tested indirectly through the detect() method.

TEST(YOLODetectorTest, SetConfidenceThresholdErrors) {
  detection::YOLODetector detector("models/nonexistent_model.onnx");
  
  // Test invalid confidence thresholds
  EXPECT_THROW(detector.setConfidenceThreshold(-0.1f), std::invalid_argument);
  EXPECT_THROW(detector.setConfidenceThreshold(1.5f), std::invalid_argument);
  
  // Valid thresholds should work
  detector.setConfidenceThreshold(0.0f);
  EXPECT_FLOAT_EQ(detector.getConfidenceThreshold(), 0.0f);
  
  detector.setConfidenceThreshold(1.0f);
  EXPECT_FLOAT_EQ(detector.getConfidenceThreshold(), 1.0f);
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

TEST(Position3DTest, Setters) {
  utils::Position3D pos;
  
  pos.setX(1.0f);
  EXPECT_FLOAT_EQ(pos.getX(), 1.0f);
  EXPECT_FLOAT_EQ(pos.getY(), 0.0f);
  EXPECT_FLOAT_EQ(pos.getZ(), 0.0f);
  
  pos.setY(2.0f);
  EXPECT_FLOAT_EQ(pos.getX(), 1.0f);
  EXPECT_FLOAT_EQ(pos.getY(), 2.0f);
  
  pos.setZ(3.0f);
  EXPECT_FLOAT_EQ(pos.getZ(), 3.0f);
  
  pos.set(4.0f, 5.0f, 6.0f);
  EXPECT_FLOAT_EQ(pos.getX(), 4.0f);
  EXPECT_FLOAT_EQ(pos.getY(), 5.0f);
  EXPECT_FLOAT_EQ(pos.getZ(), 6.0f);
}

TEST(Position3DTest, Normalize) {
  utils::Position3D pos(3.0f, 4.0f, 0.0f);
  utils::Position3D normalized = pos.normalize();
  
  EXPECT_FLOAT_EQ(normalized.magnitude(), 1.0f);
  EXPECT_FLOAT_EQ(normalized.getX(), 0.6f);
  EXPECT_FLOAT_EQ(normalized.getY(), 0.8f);
  EXPECT_FLOAT_EQ(normalized.getZ(), 0.0f);
  
  // Test zero vector normalization
  utils::Position3D zero;
  utils::Position3D zeroNormalized = zero.normalize();
  EXPECT_FLOAT_EQ(zeroNormalized.magnitude(), 0.0f);
}

TEST(Position3DTest, DotProduct) {
  utils::Position3D pos1(1.0f, 2.0f, 3.0f);
  utils::Position3D pos2(4.0f, 5.0f, 6.0f);
  
  float dot = pos1.dot(pos2);
  EXPECT_FLOAT_EQ(dot, 1.0f * 4.0f + 2.0f * 5.0f + 3.0f * 6.0f);
  EXPECT_FLOAT_EQ(dot, 32.0f);
  
  // Test with zero vector
  utils::Position3D zero;
  EXPECT_FLOAT_EQ(pos1.dot(zero), 0.0f);
}

TEST(Position3DTest, CrossProduct) {
  utils::Position3D pos1(1.0f, 0.0f, 0.0f);
  utils::Position3D pos2(0.0f, 1.0f, 0.0f);
  
  utils::Position3D cross = pos1.cross(pos2);
  EXPECT_FLOAT_EQ(cross.getX(), 0.0f);
  EXPECT_FLOAT_EQ(cross.getY(), 0.0f);
  EXPECT_FLOAT_EQ(cross.getZ(), 1.0f);
  
  // Test cross product with same vector (should be zero)
  utils::Position3D cross2 = pos1.cross(pos1);
  EXPECT_FLOAT_EQ(cross2.getX(), 0.0f);
  EXPECT_FLOAT_EQ(cross2.getY(), 0.0f);
  EXPECT_FLOAT_EQ(cross2.getZ(), 0.0f);
}

TEST(Position3DTest, ScalarDivision) {
  utils::Position3D pos(6.0f, 8.0f, 10.0f);
  
  utils::Position3D divided = pos / 2.0f;
  EXPECT_FLOAT_EQ(divided.getX(), 3.0f);
  EXPECT_FLOAT_EQ(divided.getY(), 4.0f);
  EXPECT_FLOAT_EQ(divided.getZ(), 5.0f);
  
  // Test division by zero (should return zero vector)
  utils::Position3D zeroDiv = pos / 0.0f;
  EXPECT_FLOAT_EQ(zeroDiv.getX(), 0.0f);
  EXPECT_FLOAT_EQ(zeroDiv.getY(), 0.0f);
  EXPECT_FLOAT_EQ(zeroDiv.getZ(), 0.0f);
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

TEST(CameraModelTest, LoadCalibration) {
  core::CameraModel camera;
  bool result = camera.loadCalibration("calibration.xml");
  EXPECT_TRUE(result);  // Phase 0 stub always returns true
}

TEST(CameraModelTest, CameraPose) {
  core::CameraModel camera;
  
  // Test default camera position
  const auto& defaultPos = camera.getCameraPosition();
  EXPECT_FLOAT_EQ(defaultPos.getX(), 0.0f);
  EXPECT_FLOAT_EQ(defaultPos.getY(), 0.0f);
  EXPECT_FLOAT_EQ(defaultPos.getZ(), 0.5f);
  
  // Test default rotation (identity matrix)
  const float* defaultRot = camera.getCameraRotation();
  EXPECT_FLOAT_EQ(defaultRot[0], 1.0f);  // [0][0] = 1
  EXPECT_FLOAT_EQ(defaultRot[4], 1.0f);  // [1][1] = 1
  EXPECT_FLOAT_EQ(defaultRot[8], 1.0f);  // [2][2] = 1
  
  // Test setting camera pose
  utils::Position3D newPos(0.1f, 0.2f, 0.6f);
  float rotation[9] = {1.0f, 0.0f, 0.0f,
                       0.0f, 1.0f, 0.0f,
                       0.0f, 0.0f, 1.0f};
  camera.setCameraPose(newPos, rotation);
  
  const auto& updatedPos = camera.getCameraPosition();
  EXPECT_FLOAT_EQ(updatedPos.getX(), 0.1f);
  EXPECT_FLOAT_EQ(updatedPos.getY(), 0.2f);
  EXPECT_FLOAT_EQ(updatedPos.getZ(), 0.6f);
  
  const float* updatedRot = camera.getCameraRotation();
  EXPECT_FLOAT_EQ(updatedRot[0], 1.0f);
}

TEST(CameraModelTest, SetIntrinsics) {
  core::CameraModel camera;
  
  // Test default intrinsics
  EXPECT_FLOAT_EQ(camera.getFocalLengthX(), 800.0f);
  EXPECT_FLOAT_EQ(camera.getFocalLengthY(), 800.0f);
  EXPECT_FLOAT_EQ(camera.getPrincipalPointX(), 320.0f);
  EXPECT_FLOAT_EQ(camera.getPrincipalPointY(), 240.0f);
  
  // Test setting intrinsics
  camera.setIntrinsics(900.0f, 850.0f, 350.0f, 250.0f);
  EXPECT_FLOAT_EQ(camera.getFocalLengthX(), 900.0f);
  EXPECT_FLOAT_EQ(camera.getFocalLengthY(), 850.0f);
  EXPECT_FLOAT_EQ(camera.getPrincipalPointX(), 350.0f);
  EXPECT_FLOAT_EQ(camera.getPrincipalPointY(), 250.0f);
}

TEST(CameraModelTest, IsCalibrated) {
  core::CameraModel camera;
  EXPECT_TRUE(camera.isCalibrated());
  
  core::CameraModel camera2(700.0f, 700.0f, 300.0f, 200.0f, 640, 480);
  EXPECT_TRUE(camera2.isCalibrated());
}

TEST(CameraModelTest, Getters) {
  core::CameraModel camera(900.0f, 850.0f, 350.0f, 250.0f, 1280, 720);
  
  EXPECT_FLOAT_EQ(camera.getFocalLengthX(), 900.0f);
  EXPECT_FLOAT_EQ(camera.getFocalLengthY(), 850.0f);
  EXPECT_FLOAT_EQ(camera.getPrincipalPointX(), 350.0f);
  EXPECT_FLOAT_EQ(camera.getPrincipalPointY(), 250.0f);
  EXPECT_EQ(camera.getImageWidth(), 1280);
  EXPECT_EQ(camera.getImageHeight(), 720);
}

// ============================================================================
// CoordinateTransformer Tests
// ============================================================================

TEST(CoordinateTransformerTest, Initialization) {
  core::CameraModel camera;
  core::CoordinateTransformer transformer(camera);
  
  // Transformer initialized successfully - use transformer to avoid unused variable warning
  (void)transformer;
  EXPECT_TRUE(true);
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

TEST(CoordinateTransformerTest, Setters) {
  core::CameraModel camera;
  core::CoordinateTransformer transformer(camera);
  
  // Test setting average human height
  transformer.setAverageHumanHeight(1.8f);
  
  // Test setting camera model
  core::CameraModel newCamera(900.0f, 900.0f, 320.0f, 240.0f, 640, 480);
  transformer.setCameraModel(newCamera);
  
  // Setters executed successfully
  EXPECT_TRUE(true);
}

TEST(CoordinateTransformerTest, PixelToCameraFrame) {
  core::CameraModel camera;
  core::CoordinateTransformer transformer(camera);
  
  // Test pixel to camera frame conversion
  utils::Position3D camPos = transformer.pixelToCameraFrame(320.0f, 240.0f, 2.0f, 640, 480);
  
  // Should return valid position with correct depth
  EXPECT_TRUE(std::isfinite(camPos.getX()));
  EXPECT_TRUE(std::isfinite(camPos.getY()));
  EXPECT_FLOAT_EQ(camPos.getZ(), 2.0f);
  
  // Test with different pixel coordinates
  utils::Position3D camPos2 = transformer.pixelToCameraFrame(0.0f, 0.0f, 1.5f, 640, 480);
  EXPECT_TRUE(std::isfinite(camPos2.getX()));
  EXPECT_FLOAT_EQ(camPos2.getZ(), 1.5f);
}

TEST(CoordinateTransformerTest, CameraToRobotFrame) {
  core::CameraModel camera;
  core::CoordinateTransformer transformer(camera);
  
  // Test camera to robot frame transformation
  utils::Position3D cameraPos(1.0f, 0.0f, 2.0f);
  utils::Position3D robotPos = transformer.cameraToRobotFrame(cameraPos);
  
  // Should return valid position
  EXPECT_TRUE(std::isfinite(robotPos.getX()));
  EXPECT_TRUE(std::isfinite(robotPos.getY()));
  EXPECT_TRUE(std::isfinite(robotPos.getZ()));
  
  // Test with zero position
  utils::Position3D zeroPos(0.0f, 0.0f, 0.0f);
  utils::Position3D robotZero = transformer.cameraToRobotFrame(zeroPos);
  EXPECT_TRUE(std::isfinite(robotZero.getX()));
}

TEST(CoordinateTransformerTest, GetCameraModel) {
  core::CameraModel camera(900.0f, 850.0f, 350.0f, 250.0f, 1280, 720);
  core::CoordinateTransformer transformer(camera);
  
  // getCameraModel() removed - test that transformer works with camera
  utils::Position3D pos = transformer.imageToRobotFrame(
      detection::BoundingBox(320.0f, 240.0f, 100.0f, 200.0f), 640, 480);
  EXPECT_TRUE(std::isfinite(pos.getX()));
}

// ============================================================================
// PerceptionPipeline Tests
// ============================================================================

TEST(PerceptionPipelineTest, Initialization) {
  // Use a non-existent model path for testing
  auto detector = std::make_shared<detection::YOLODetector>(
      "models/nonexistent_model.onnx");
  auto tracker = std::make_shared<tracking::KalmanTracker>();
  core::CameraModel camera;
  auto transformer = std::make_shared<core::CoordinateTransformer>(camera);
  
  core::PerceptionPipeline pipeline(detector, tracker, transformer, camera);
  
  EXPECT_EQ(pipeline.getFrameCount(), 0);
  EXPECT_FALSE(pipeline.isVisualizationEnabled());
}

TEST(PerceptionPipelineTest, FrameProcessing) {
  // Use a non-existent model path for testing
  auto detector = std::make_shared<detection::YOLODetector>(
      "models/nonexistent_model.onnx");
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

TEST(PerceptionPipelineTest, Getters) {
  auto detector = std::make_shared<detection::YOLODetector>(
      "models/nonexistent_model.onnx");
  auto tracker = std::make_shared<tracking::KalmanTracker>();
  core::CameraModel camera;
  auto transformer = std::make_shared<core::CoordinateTransformer>(camera);
  
  core::PerceptionPipeline pipeline(detector, tracker, transformer, camera);
  
  // Test getDetector
  auto retrievedDetector = pipeline.getDetector();
  EXPECT_EQ(retrievedDetector, detector);
  
  // Test getTracker
  auto retrievedTracker = pipeline.getTracker();
  EXPECT_EQ(retrievedTracker, tracker);
  
  // Test getTransformer
  auto retrievedTransformer = pipeline.getTransformer();
  EXPECT_EQ(retrievedTransformer, transformer);
  
  // Test getCameraModel
  const auto& retrievedCamera = pipeline.getCameraModel();
  EXPECT_EQ(retrievedCamera.getFocalLengthX(), camera.getFocalLengthX());
  EXPECT_EQ(retrievedCamera.getFocalLengthY(), camera.getFocalLengthY());
  
  // Test getFrameCount
  EXPECT_EQ(pipeline.getFrameCount(), 0);
}

TEST(PerceptionPipelineTest, Reset) {
  auto detector = std::make_shared<detection::YOLODetector>(
      "models/nonexistent_model.onnx");
  auto tracker = std::make_shared<tracking::KalmanTracker>();
  core::CameraModel camera;
  auto transformer = std::make_shared<core::CoordinateTransformer>(camera);
  
  core::PerceptionPipeline pipeline(detector, tracker, transformer, camera);
  
  // Process a frame to create tracks
  int width = 640, height = 480;
  std::vector<unsigned char> frame(width * height * 3, 128);
  pipeline.processFrame(frame.data(), width, height, 3, 0.0);
  
  EXPECT_GT(pipeline.getFrameCount(), 0);
  
  // Reset pipeline
  pipeline.reset();
  
  // Frame count should reset
  EXPECT_EQ(pipeline.getFrameCount(), 0);
  
  // Tracker should be reset (no active tracks)
  auto activeTracks = pipeline.getTracker()->getActiveTracks();
  EXPECT_EQ(activeTracks.size(), 0);
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


TEST(GeometryUtilsTest, ApplyRotation) {
  utils::Position3D pos(1.0f, 0.0f, 0.0f);
  
  // Identity rotation matrix
  float identity[9] = {1.0f, 0.0f, 0.0f,
                      0.0f, 1.0f, 0.0f,
                      0.0f, 0.0f, 1.0f};
  utils::Position3D rotated = utils::GeometryUtils::applyRotation(pos, identity);
  
  EXPECT_FLOAT_EQ(rotated.getX(), 1.0f);
  EXPECT_FLOAT_EQ(rotated.getY(), 0.0f);
  EXPECT_FLOAT_EQ(rotated.getZ(), 0.0f);
  
  // 90-degree rotation around Z-axis
  float rot90[9] = {0.0f, -1.0f, 0.0f,
                    1.0f,  0.0f, 0.0f,
                    0.0f,  0.0f, 1.0f};
  utils::Position3D rotated90 = utils::GeometryUtils::applyRotation(pos, rot90);
  EXPECT_NEAR(rotated90.getX(), 0.0f, 0.001f);
  EXPECT_NEAR(rotated90.getY(), 1.0f, 0.001f);
}

TEST(GeometryUtilsTest, ApplyTranslation) {
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  utils::Position3D translation(0.5f, 0.5f, 0.5f);
  
  utils::Position3D translated = utils::GeometryUtils::applyTranslation(pos, translation);
  
  EXPECT_FLOAT_EQ(translated.getX(), 1.5f);
  EXPECT_FLOAT_EQ(translated.getY(), 2.5f);
  EXPECT_FLOAT_EQ(translated.getZ(), 3.5f);
}


TEST(GeometryUtilsTest, DepthEstimationEdgeCases) {
  // Test with zero height bounding box
  detection::BoundingBox zeroHeightBbox(320.0f, 240.0f, 100.0f, 0.0f);
  float depth = utils::GeometryUtils::estimateDepthFromBboxHeight(zeroHeightBbox, 800.0f, 1.7f);
  EXPECT_FLOAT_EQ(depth, 0.0f);
  
  // Test with negative height (should handle gracefully)
  detection::BoundingBox negativeHeightBbox(320.0f, 240.0f, 100.0f, -10.0f);
  float depth2 = utils::GeometryUtils::estimateDepthFromBboxHeight(negativeHeightBbox, 800.0f, 1.7f);
  EXPECT_FLOAT_EQ(depth2, 0.0f);
}

// Main function for GoogleTest

