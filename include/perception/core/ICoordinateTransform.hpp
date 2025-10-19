/**
 * @file ICoordinateTransform.hpp
 * @brief Interface for coordinate transformation operations
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include "../utils/Position3D.hpp"
#include "../detection/BoundingBox.hpp"

namespace perception {
namespace core {

/**
 * @interface ICoordinateTransform
 * @brief Abstract interface for coordinate transformations
 * 
 * This interface defines coordinate transformation operations from
 * image coordinates to robot reference frame. Follows Strategy pattern.
 */
class ICoordinateTransform {
 public:
  /**
   * @brief Virtual destructor for proper cleanup
   */
  virtual ~ICoordinateTransform() = default;

  /**
   * @brief Transform 2D bounding box to 3D position in robot frame
   * 
   * Uses camera intrinsics and bounding box dimensions to estimate
   * depth and compute 3D position.
   * 
   * @param bbox Bounding box in image coordinates
   * @param imageWidth Width of the image in pixels
   * @param imageHeight Height of the image in pixels
   * @return utils::Position3D Estimated 3D position in robot frame (meters)
   */
  virtual utils::Position3D imageToRobotFrame(
      const detection::BoundingBox& bbox,
      int imageWidth,
      int imageHeight) const = 0;

  /**
   * @brief Transform pixel coordinates to camera frame
   * 
   * @param pixelX X coordinate in image (pixels)
   * @param pixelY Y coordinate in image (pixels)
   * @param depth Estimated depth (meters)
   * @param imageWidth Width of the image in pixels
   * @param imageHeight Height of the image in pixels
   * @return utils::Position3D Position in camera frame (meters)
   */
  virtual utils::Position3D pixelToCameraFrame(
      float pixelX,
      float pixelY,
      float depth,
      int imageWidth,
      int imageHeight) const = 0;

  /**
   * @brief Transform camera frame coordinates to robot base frame
   * 
   * @param cameraPos Position in camera frame
   * @return utils::Position3D Position in robot frame (meters)
   */
  virtual utils::Position3D cameraToRobotFrame(
      const utils::Position3D& cameraPos) const = 0;
};

}  // namespace core
}  // namespace perception

