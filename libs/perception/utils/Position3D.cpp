/**
 * @file Position3D.cpp
 * @brief Implementation of Position3D class
 * @author Acme Robotics
 */

#include "perception/utils/Position3D.hpp"
#include <cmath>

namespace perception {
namespace utils {

Position3D::Position3D() : x_(0.0f), y_(0.0f), z_(0.0f) {}

Position3D::Position3D(float x, float y, float z) : x_(x), y_(y), z_(z) {}

float Position3D::getX() const {
  return x_;
}

float Position3D::getY() const {
  return y_;
}

float Position3D::getZ() const {
  return z_;
}

void Position3D::setX(float x) {
  x_ = x;
}

void Position3D::setY(float y) {
  y_ = y;
}

void Position3D::setZ(float z) {
  z_ = z;
}

void Position3D::set(float x, float y, float z) {
  x_ = x;
  y_ = y;
  z_ = z;
}

float Position3D::distanceTo(const Position3D& other) const {
  float dx = x_ - other.x_;
  float dy = y_ - other.y_;
  float dz = z_ - other.z_;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float Position3D::magnitude() const {
  return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
}

Position3D Position3D::normalize() const {
  float mag = magnitude();
  if (mag > 0.0f) {
    return Position3D(x_ / mag, y_ / mag, z_ / mag);
  }
  return Position3D(0.0f, 0.0f, 0.0f);
}

Position3D Position3D::operator+(const Position3D& other) const {
  return Position3D(x_ + other.x_, y_ + other.y_, z_ + other.z_);
}

Position3D Position3D::operator-(const Position3D& other) const {
  return Position3D(x_ - other.x_, y_ - other.y_, z_ - other.z_);
}

Position3D Position3D::operator*(float scalar) const {
  return Position3D(x_ * scalar, y_ * scalar, z_ * scalar);
}

Position3D Position3D::operator/(float scalar) const {
  if (scalar != 0.0f) {
    return Position3D(x_ / scalar, y_ / scalar, z_ / scalar);
  }
  return Position3D(0.0f, 0.0f, 0.0f);
}

float Position3D::dot(const Position3D& other) const {
  return x_ * other.x_ + y_ * other.y_ + z_ * other.z_;
}

Position3D Position3D::cross(const Position3D& other) const {
  return Position3D(
      y_ * other.z_ - z_ * other.y_,
      z_ * other.x_ - x_ * other.z_,
      x_ * other.y_ - y_ * other.x_
  );
}

}  // namespace utils
}  // namespace perception

