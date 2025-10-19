/**
 * @file CoordinateTransformer.hpp
 * @brief Coordinate transformation implementation
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include "ICoordinateTransform.hpp"
#include "CameraModel.hpp"
#include "../utils/Position3D.hpp"
#include "../detection/BoundingBox.hpp"

namespace perception {
namespace core {

/**
 * @class CoordinateTransformer
 * @brief Implements coordinate transformations from image to robot frame
 * 
 * Transforms 2D bounding boxes in image coordinates to 3D positions
 * in the robot's reference frame using camera model and depth estimation.
 */
class CoordinateTransformer : public ICoordinateTransform {
 public:
  /**
   * @brief Construct transformer with camera model
   * 
   * @param cameraModel Camera intrinsic and extrinsic parameters
   */
  explicit CoordinateTransformer(const CameraModel& cameraModel);

  /**
   * @brief Transform 2D bounding box to 3D position in robot frame
   * 
   * Steps:
   * 1. Estimate depth from bounding box height
   * 2. Convert pixel coordinates to camera frame
   * 3. Transform from camera frame to robot frame
   * 
   * @param bbox Bounding box in image coordinates
   * @param imageWidth Width of the image in pixels
   * @param imageHeight Height of the image in pixels
   * @return utils::Position3D Estimated 3D position in robot frame (meters)
   */
  utils::Position3D imageToRobotFrame(
      const detection::BoundingBox& bbox,
      int imageWidth,
      int imageHeight) const override;

  /**
   * @brief Transform pixel coordinates to camera frame
   * 
   * Uses pinhole camera model with camera intrinsics.
   * 
   * @param pixelX X coordinate in image (pixels)
   * @param pixelY Y coordinate in image (pixels)
   * @param depth Estimated depth (meters)
   * @param imageWidth Width of the image in pixels
   * @param imageHeight Height of the image in pixels
   * @return utils::Position3D Position in camera frame (meters)
   */
  utils::Position3D pixelToCameraFrame(
      float pixelX,
      float pixelY,
      float depth,
      int imageWidth,
      int imageHeight) const override;

  /**
   * @brief Transform camera frame coordinates to robot base frame
   * 
   * Applies rotation and translation from camera pose.
   * 
   * @param cameraPos Position in camera frame
   * @return utils::Position3D Position in robot frame (meters)
   */
  utils::Position3D cameraToRobotFrame(
      const utils::Position3D& cameraPos) const override;

  /**
   * @brief Set camera model
   * 
   * @param cameraModel New camera model
   */
  void setCameraModel(const CameraModel& cameraModel);

  /**
   * @brief Get current camera model
   * 
   * @return const CameraModel& Camera model reference
   */
  const CameraModel& getCameraModel() const;

  /**
   * @brief Set average human height for depth estimation
   * 
   * @param height Average height in meters (default 1.7m)
   */
  void setAverageHumanHeight(float height);

  /**
   * @brief Get average human height used for depth estimation
   * 
   * @return float Height in meters
   */
  float getAverageHumanHeight() const;

 private:
  CameraModel cameraModel_;          ///< Camera parameters
  float averageHumanHeight_;         ///< Assumed human height (m)
};

}  // namespace core
}  // namespace perception

