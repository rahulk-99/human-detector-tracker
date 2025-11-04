/**
 * @file CoordinateTransformer.cpp
 * @brief Implementation of CoordinateTransformer class
 * @author Acme Robotics
 */

#include "perception/core/CoordinateTransformer.hpp"
#include "perception/utils/GeometryUtils.hpp"

namespace perception {
namespace core {

CoordinateTransformer::CoordinateTransformer(const CameraModel& cameraModel)
    : cameraModel_(cameraModel), averageHumanHeight_(1.7f) {}

utils::Position3D CoordinateTransformer::imageToRobotFrame(
    const detection::BoundingBox& bbox,
    int imageWidth,
    int imageHeight) const {
  // Step 1: Estimate depth from bounding box height
  float focalLength = cameraModel_.getFocalLengthY();
  float depth = utils::GeometryUtils::estimateDepthFromBboxHeight(
      bbox, focalLength, averageHumanHeight_);

  // Step 2: Convert bbox center to camera frame
  float pixelX = bbox.getX();
  float pixelY = bbox.getY();
  utils::Position3D cameraPos = pixelToCameraFrame(
      pixelX, pixelY, depth, imageWidth, imageHeight);

  // Step 3: Transform to robot frame
  utils::Position3D robotPos = cameraToRobotFrame(cameraPos);

  return robotPos;
}

utils::Position3D CoordinateTransformer::pixelToCameraFrame(
    float pixelX,
    float pixelY,
    float depth,
    int imageWidth,
    int imageHeight) const {
  // Pinhole camera model:
  // X = (u - cx) * Z / fx
  // Y = (v - cy) * Z / fy
  // Z = depth

  float cx = cameraModel_.getPrincipalPointX();
  float cy = cameraModel_.getPrincipalPointY();
  float fx = cameraModel_.getFocalLengthX();
  float fy = cameraModel_.getFocalLengthY();

  float X = (pixelX - cx) * depth / fx;
  float Y = (pixelY - cy) * depth / fy;
  float Z = depth;

  return utils::Position3D(X, Y, Z);
}

utils::Position3D CoordinateTransformer::cameraToRobotFrame(
    const utils::Position3D& cameraPos) const {
  // Apply rotation from camera to robot frame
  const float* R = cameraModel_.getCameraRotation();
  utils::Position3D rotated = utils::GeometryUtils::applyRotation(cameraPos, R);

  // Apply translation (camera position relative to robot base)
  utils::Position3D robotPos = utils::GeometryUtils::applyTranslation(
      rotated, cameraModel_.getCameraPosition());

  return robotPos;
}

void CoordinateTransformer::setCameraModel(const CameraModel& cameraModel) {
  cameraModel_ = cameraModel;
}

void CoordinateTransformer::setAverageHumanHeight(float height) {
  averageHumanHeight_ = height;
}

}  // namespace core
}  // namespace perception

