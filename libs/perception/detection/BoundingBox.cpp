/**
 * @file BoundingBox.cpp
 * @brief Implementation of BoundingBox class
 * @author Acme Robotics
 */

#include "perception/detection/BoundingBox.hpp"
#include <algorithm>

namespace perception {
namespace detection {

BoundingBox::BoundingBox() : x_(0.0f), y_(0.0f), width_(0.0f), height_(0.0f) {}

BoundingBox::BoundingBox(float x, float y, float width, float height)
    : x_(x), y_(y), width_(width), height_(height) {}

float BoundingBox::getX() const {
  return x_;
}

float BoundingBox::getY() const {
  return y_;
}

float BoundingBox::getWidth() const {
  return width_;
}

float BoundingBox::getHeight() const {
  return height_;
}

float BoundingBox::getLeft() const {
  return x_ - width_ / 2.0f;
}

float BoundingBox::getRight() const {
  return x_ + width_ / 2.0f;
}

float BoundingBox::getTop() const {
  return y_ - height_ / 2.0f;
}

float BoundingBox::getBottom() const {
  return y_ + height_ / 2.0f;
}

float BoundingBox::getArea() const {
  return width_ * height_;
}

float BoundingBox::computeIoU(const BoundingBox& other) const {
  // Compute intersection area
  float left = std::max(getLeft(), other.getLeft());
  float right = std::min(getRight(), other.getRight());
  float top = std::max(getTop(), other.getTop());
  float bottom = std::min(getBottom(), other.getBottom());

  float intersectionWidth = std::max(0.0f, right - left);
  float intersectionHeight = std::max(0.0f, bottom - top);
  float intersectionArea = intersectionWidth * intersectionHeight;

  // Compute union area
  float unionArea = getArea() + other.getArea() - intersectionArea;

  // Compute IoU
  if (unionArea <= 0.0f) {
    return 0.0f;
  }
  return intersectionArea / unionArea;
}

void BoundingBox::set(float x, float y, float width, float height) {
  x_ = x;
  y_ = y;
  width_ = width;
  height_ = height;
}

}  // namespace detection
}  // namespace perception

