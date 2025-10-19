/**
 * @file GeometryUtils.cpp
 * @brief Implementation of GeometryUtils class
 * @author Acme Robotics
 */

#include "perception/utils/GeometryUtils.hpp"
#include <algorithm>
#include <cmath>

namespace perception {
namespace utils {

float GeometryUtils::estimateDepthFromBboxHeight(
    const detection::BoundingBox& bbox,
    float focalLength,
    float averageHumanHeight) {
  float pixelHeight = bbox.getHeight();
  
  if (pixelHeight <= 0.0f) {
    return 0.0f;
  }

  // Pinhole camera model: depth = (focal * real_height) / pixel_height
  float depth = (focalLength * averageHumanHeight) / pixelHeight;
  
  return depth;
}

float GeometryUtils::computeIntersectionArea(
    const detection::BoundingBox& box1,
    const detection::BoundingBox& box2) {
  float left = std::max(box1.getLeft(), box2.getLeft());
  float right = std::min(box1.getRight(), box2.getRight());
  float top = std::max(box1.getTop(), box2.getTop());
  float bottom = std::min(box1.getBottom(), box2.getBottom());

  float width = std::max(0.0f, right - left);
  float height = std::max(0.0f, bottom - top);

  return width * height;
}

float GeometryUtils::computeUnionArea(
    const detection::BoundingBox& box1,
    const detection::BoundingBox& box2) {
  float intersectionArea = computeIntersectionArea(box1, box2);
  return box1.getArea() + box2.getArea() - intersectionArea;
}

Position3D GeometryUtils::applyRotation(
    const Position3D& position,
    const float* rotationMatrix) {
  float x = position.getX();
  float y = position.getY();
  float z = position.getZ();

  // Matrix multiplication: R * [x; y; z]
  float newX = rotationMatrix[0] * x + rotationMatrix[1] * y +
               rotationMatrix[2] * z;
  float newY = rotationMatrix[3] * x + rotationMatrix[4] * y +
               rotationMatrix[5] * z;
  float newZ = rotationMatrix[6] * x + rotationMatrix[7] * y +
               rotationMatrix[8] * z;

  return Position3D(newX, newY, newZ);
}

Position3D GeometryUtils::applyTranslation(
    const Position3D& position,
    const Position3D& translation) {
  return position + translation;
}

void GeometryUtils::pixelToNormalized(float pixelX, float pixelY,
                                     int imageWidth, int imageHeight,
                                     float& normX, float& normY) {
  normX = (2.0f * pixelX / imageWidth) - 1.0f;
  normY = (2.0f * pixelY / imageHeight) - 1.0f;
}

float GeometryUtils::clamp(float value, float min, float max) {
  return std::max(min, std::min(value, max));
}

}  // namespace utils
}  // namespace perception

