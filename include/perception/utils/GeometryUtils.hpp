/**
 * @file GeometryUtils.hpp
 * @brief Geometric utility functions
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include "../detection/BoundingBox.hpp"
#include "Position3D.hpp"

namespace perception {
namespace utils {

/**
 * @class GeometryUtils
 * @brief Static utility functions for geometric operations
 * 
 * Provides helper functions for bounding box operations,
 * depth estimation, and coordinate transformations.
 */
class GeometryUtils {
 public:
  /**
   * @brief Estimate depth from bounding box height
   * 
   * Uses pinhole camera model and assumed average human height
   * to estimate distance to person from bounding box height in pixels.
   * 
   * Formula: depth = (focal_length * real_height) / pixel_height
   * 
   * @param bbox Bounding box of detected person
   * @param focalLength Camera focal length in pixels
   * @param averageHumanHeight Assumed average human height (meters, default 1.7m)
   * @return float Estimated depth in meters
   */
  static float estimateDepthFromBboxHeight(
      const detection::BoundingBox& bbox,
      float focalLength,
      float averageHumanHeight = 1.7f);

  /**
   * @brief Compute intersection area of two bounding boxes
   * 
   * @param box1 First bounding box
   * @param box2 Second bounding box
   * @return float Intersection area in square pixels
   */
  static float computeIntersectionArea(
      const detection::BoundingBox& box1,
      const detection::BoundingBox& box2);

  /**
   * @brief Compute union area of two bounding boxes
   * 
   * @param box1 First bounding box
   * @param box2 Second bounding box
   * @return float Union area in square pixels
   */
  static float computeUnionArea(
      const detection::BoundingBox& box1,
      const detection::BoundingBox& box2);

  /**
   * @brief Apply 3D rotation matrix to position
   * 
   * @param position Input position
   * @param rotationMatrix 3x3 rotation matrix (row-major, 9 elements)
   * @return Position3D Rotated position
   */
  static Position3D applyRotation(
      const Position3D& position,
      const float* rotationMatrix);

  /**
   * @brief Apply 3D translation to position
   * 
   * @param position Input position
   * @param translation Translation vector
   * @return Position3D Translated position
   */
  static Position3D applyTranslation(
      const Position3D& position,
      const Position3D& translation);

  /**
   * @brief Convert pixel coordinates to normalized image coordinates
   * 
   * Normalizes pixel coordinates to [-1, 1] range based on image dimensions.
   * 
   * @param pixelX Pixel X coordinate
   * @param pixelY Pixel Y coordinate
   * @param imageWidth Image width in pixels
   * @param imageHeight Image height in pixels
   * @param normX Output normalized X coordinate
   * @param normY Output normalized Y coordinate
   */
  static void pixelToNormalized(float pixelX, float pixelY,
                               int imageWidth, int imageHeight,
                               float& normX, float& normY);

  /**
   * @brief Clamp value to range [min, max]
   * 
   * @param value Input value
   * @param min Minimum bound
   * @param max Maximum bound
   * @return float Clamped value
   */
  static float clamp(float value, float min, float max);

 private:
  GeometryUtils() = delete;  ///< Utility class, no instantiation
};

}  // namespace utils
}  // namespace perception

