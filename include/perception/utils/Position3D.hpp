/**
 * @file Position3D.hpp
 * @brief 3D position/vector representation
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include <cmath>

namespace perception {
namespace utils {

/**
 * @class Position3D
 * @brief Represents a 3D position or velocity vector
 * 
 * Can represent positions in meters or velocities in m/s.
 * Provides common vector operations.
 */
class Position3D {
 public:
  /**
   * @brief Default constructor - initializes to origin
   */
  Position3D();

  /**
   * @brief Construct with coordinates
   * 
   * @param x X coordinate (meters or m/s)
   * @param y Y coordinate (meters or m/s)
   * @param z Z coordinate (meters or m/s)
   */
  Position3D(float x, float y, float z);

  /**
   * @brief Get X coordinate
   * @return float X value
   */
  float getX() const;

  /**
   * @brief Get Y coordinate
   * @return float Y value
   */
  float getY() const;

  /**
   * @brief Get Z coordinate
   * @return float Z value
   */
  float getZ() const;

  /**
   * @brief Set X coordinate
   * @param x New X value
   */
  void setX(float x);

  /**
   * @brief Set Y coordinate
   * @param y New Y value
   */
  void setY(float y);

  /**
   * @brief Set Z coordinate
   * @param z New Z value
   */
  void setZ(float z);

  /**
   * @brief Set all coordinates
   * @param x X value
   * @param y Y value
   * @param z Z value
   */
  void set(float x, float y, float z);

  /**
   * @brief Calculate Euclidean distance to another position
   * 
   * @param other Another position
   * @return float Distance in same units as coordinates
   */
  float distanceTo(const Position3D& other) const;

  /**
   * @brief Calculate magnitude (norm) of vector
   * 
   * @return float Vector magnitude
   */
  float magnitude() const;

  /**
   * @brief Normalize vector to unit length
   * 
   * @return Position3D Normalized vector
   */
  Position3D normalize() const;

  /**
   * @brief Vector addition operator
   * 
   * @param other Vector to add
   * @return Position3D Result of addition
   */
  Position3D operator+(const Position3D& other) const;

  /**
   * @brief Vector subtraction operator
   * 
   * @param other Vector to subtract
   * @return Position3D Result of subtraction
   */
  Position3D operator-(const Position3D& other) const;

  /**
   * @brief Scalar multiplication operator
   * 
   * @param scalar Scalar value
   * @return Position3D Scaled vector
   */
  Position3D operator*(float scalar) const;

  /**
   * @brief Scalar division operator
   * 
   * @param scalar Scalar divisor
   * @return Position3D Scaled vector
   */
  Position3D operator/(float scalar) const;

  /**
   * @brief Dot product with another vector
   * 
   * @param other Another vector
   * @return float Dot product result
   */
  float dot(const Position3D& other) const;

  /**
   * @brief Cross product with another vector
   * 
   * @param other Another vector
   * @return Position3D Cross product result
   */
  Position3D cross(const Position3D& other) const;

 private:
  float x_;  ///< X coordinate
  float y_;  ///< Y coordinate
  float z_;  ///< Z coordinate
};

}  // namespace utils
}  // namespace perception

