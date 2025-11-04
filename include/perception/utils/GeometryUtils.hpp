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

 private:
  GeometryUtils() = delete;  ///< Utility class, no instantiation
};

}  // namespace utils
}  // namespace perception

