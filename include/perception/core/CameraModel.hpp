/**
 * @file CameraModel.hpp
 * @brief Camera model and intrinsic parameters
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include <string>
#include "../utils/Position3D.hpp"

namespace perception {
namespace core {

/**
 * @class CameraModel
 * @brief Represents camera intrinsic and extrinsic parameters
 * 
 * Stores camera calibration data including focal length, principal point,
 * distortion coefficients, and camera pose relative to robot base frame.
 * Uses pinhole camera model.
 */
class CameraModel {
 public:
  /**
   * @brief Default constructor with typical camera parameters
   */
  CameraModel();

  /**
   * @brief Construct with camera intrinsics
   * 
   * @param fx Focal length in X direction (pixels)
   * @param fy Focal length in Y direction (pixels)
   * @param cx Principal point X coordinate (pixels)
   * @param cy Principal point Y coordinate (pixels)
   * @param imageWidth Image width in pixels
   * @param imageHeight Image height in pixels
   */
  CameraModel(float fx, float fy, float cx, float cy,
             int imageWidth, int imageHeight);

  /**
   * @brief Load camera parameters from calibration file
   * 
   * @param calibrationFile Path to calibration file (YAML format)
   * @return true if loaded successfully
   * @return false if loading failed
   */
  bool loadCalibration(const std::string& calibrationFile);

  /**
   * @brief Get focal length in X direction
   * @return float Focal length X (pixels)
   */
  float getFocalLengthX() const;

  /**
   * @brief Get focal length in Y direction
   * @return float Focal length Y (pixels)
   */
  float getFocalLengthY() const;

  /**
   * @brief Get principal point X coordinate
   * @return float Principal point X (pixels)
   */
  float getPrincipalPointX() const;

  /**
   * @brief Get principal point Y coordinate
   * @return float Principal point Y (pixels)
   */
  float getPrincipalPointY() const;

  /**
   * @brief Get image width
   * @return int Image width (pixels)
   */
  int getImageWidth() const;

  /**
   * @brief Get image height
   * @return int Image height (pixels)
   */
  int getImageHeight() const;

  /**
   * @brief Get camera position relative to robot base frame
   * @return const utils::Position3D& Camera position (meters)
   */
  const utils::Position3D& getCameraPosition() const;

  /**
   * @brief Get camera orientation (rotation matrix)
   * 
   * @return const float* Pointer to 3x3 rotation matrix (row-major)
   */
  const float* getCameraRotation() const;

  /**
   * @brief Set camera pose relative to robot base
   * 
   * @param position Camera position (x, y, z) in meters
   * @param rotationMatrix 3x3 rotation matrix (row-major, 9 elements)
   */
  void setCameraPose(const utils::Position3D& position,
                    const float* rotationMatrix);

  /**
   * @brief Set camera intrinsics
   * 
   * @param fx Focal length X
   * @param fy Focal length Y
   * @param cx Principal point X
   * @param cy Principal point Y
   */
  void setIntrinsics(float fx, float fy, float cx, float cy);

  /**
   * @brief Check if camera is calibrated
   * 
   * @return true if calibration data is valid
   */
  bool isCalibrated() const;

 private:
  // Intrinsic parameters
  float fx_;           ///< Focal length X (pixels)
  float fy_;           ///< Focal length Y (pixels)
  float cx_;           ///< Principal point X (pixels)
  float cy_;           ///< Principal point Y (pixels)
  int imageWidth_;     ///< Image width (pixels)
  int imageHeight_;    ///< Image height (pixels)

  // Extrinsic parameters (camera pose relative to robot base)
  utils::Position3D cameraPosition_;  ///< Camera position (m)
  float cameraRotation_[9];          ///< 3x3 rotation matrix (row-major)

  bool calibrated_;    ///< Calibration status flag
};

}  // namespace core
}  // namespace perception

