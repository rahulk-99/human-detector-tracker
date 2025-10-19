/**
 * @file CameraModel.cpp
 * @brief Implementation of CameraModel class
 * @author Acme Robotics
 */

#include "perception/core/CameraModel.hpp"
#include <iostream>

namespace perception {
namespace core {

CameraModel::CameraModel()
    : fx_(800.0f),
      fy_(800.0f),
      cx_(320.0f),
      cy_(240.0f),
      imageWidth_(640),
      imageHeight_(480),
      cameraPosition_(0.0f, 0.0f, 0.5f),  // 0.5m above robot base
      calibrated_(true) {
  // Initialize rotation matrix to identity (camera aligned with robot)
  for (int i = 0; i < 9; ++i) {
    cameraRotation_[i] = (i % 4 == 0) ? 1.0f : 0.0f;
  }
}

CameraModel::CameraModel(float fx, float fy, float cx, float cy,
                        int imageWidth, int imageHeight)
    : fx_(fx),
      fy_(fy),
      cx_(cx),
      cy_(cy),
      imageWidth_(imageWidth),
      imageHeight_(imageHeight),
      cameraPosition_(0.0f, 0.0f, 0.5f),
      calibrated_(true) {
  // Initialize rotation matrix to identity
  for (int i = 0; i < 9; ++i) {
    cameraRotation_[i] = (i % 4 == 0) ? 1.0f : 0.0f;
  }
}

bool CameraModel::loadCalibration(const std::string& calibrationFile) {
  // Phase 0: Stub - just print message
  std::cout << "[CameraModel] Loading calibration from: " << calibrationFile
            << std::endl;
  std::cout << "[CameraModel] Phase 0 - Using default parameters" << std::endl;
  return true;
}

float CameraModel::getFocalLengthX() const {
  return fx_;
}

float CameraModel::getFocalLengthY() const {
  return fy_;
}

float CameraModel::getPrincipalPointX() const {
  return cx_;
}

float CameraModel::getPrincipalPointY() const {
  return cy_;
}

int CameraModel::getImageWidth() const {
  return imageWidth_;
}

int CameraModel::getImageHeight() const {
  return imageHeight_;
}

const utils::Position3D& CameraModel::getCameraPosition() const {
  return cameraPosition_;
}

const float* CameraModel::getCameraRotation() const {
  return cameraRotation_;
}

void CameraModel::setCameraPose(const utils::Position3D& position,
                               const float* rotationMatrix) {
  cameraPosition_ = position;
  for (int i = 0; i < 9; ++i) {
    cameraRotation_[i] = rotationMatrix[i];
  }
}

void CameraModel::setIntrinsics(float fx, float fy, float cx, float cy) {
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
}

bool CameraModel::isCalibrated() const {
  return calibrated_;
}

}  // namespace core
}  // namespace perception

