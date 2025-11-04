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

}  // namespace utils
}  // namespace perception

