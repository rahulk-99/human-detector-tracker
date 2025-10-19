/**
 * @file BoundingBox.hpp
 * @brief 2D Bounding box representation
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

namespace perception {
namespace detection {

/**
 * @class BoundingBox
 * @brief Represents a 2D rectangular bounding box in image coordinates
 * 
 * Uses center-based representation (x, y, width, height) where (x,y) is
 * the center of the box. All values are in pixels.
 */
class BoundingBox {
 public:
  /**
   * @brief Default constructor - creates empty bounding box
   */
  BoundingBox();

  /**
   * @brief Construct bounding box with specified parameters
   * 
   * @param x Center X coordinate (pixels)
   * @param y Center Y coordinate (pixels)
   * @param width Width of box (pixels)
   * @param height Height of box (pixels)
   */
  BoundingBox(float x, float y, float width, float height);

  /**
   * @brief Get center X coordinate
   * @return float Center X in pixels
   */
  float getX() const;

  /**
   * @brief Get center Y coordinate
   * @return float Center Y in pixels
   */
  float getY() const;

  /**
   * @brief Get width of bounding box
   * @return float Width in pixels
   */
  float getWidth() const;

  /**
   * @brief Get height of bounding box
   * @return float Height in pixels
   */
  float getHeight() const;

  /**
   * @brief Get left edge X coordinate
   * @return float Left X coordinate
   */
  float getLeft() const;

  /**
   * @brief Get right edge X coordinate
   * @return float Right X coordinate
   */
  float getRight() const;

  /**
   * @brief Get top edge Y coordinate
   * @return float Top Y coordinate
   */
  float getTop() const;

  /**
   * @brief Get bottom edge Y coordinate
   * @return float Bottom Y coordinate
   */
  float getBottom() const;

  /**
   * @brief Calculate area of bounding box
   * @return float Area in square pixels
   */
  float getArea() const;

  /**
   * @brief Calculate Intersection over Union (IoU) with another box
   * 
   * @param other Another bounding box
   * @return float IoU score [0.0, 1.0]
   */
  float computeIoU(const BoundingBox& other) const;

  /**
   * @brief Set bounding box parameters
   * 
   * @param x Center X coordinate
   * @param y Center Y coordinate
   * @param width Width of box
   * @param height Height of box
   */
  void set(float x, float y, float width, float height);

 private:
  float x_;       ///< Center X coordinate (pixels)
  float y_;       ///< Center Y coordinate (pixels)
  float width_;   ///< Box width (pixels)
  float height_;  ///< Box height (pixels)
};

}  // namespace detection
}  // namespace perception

