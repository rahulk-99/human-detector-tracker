/**
 * @file Detection.hpp
 * @brief Detection result data structure
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include <string>
#include "BoundingBox.hpp"

namespace perception {
namespace detection {

/**
 * @class Detection
 * @brief Represents a single object detection result
 * 
 * Contains bounding box, confidence score, class label, and class ID
 * for a detected object in an image frame.
 */
class Detection {
 public:
  /**
   * @brief Default constructor
   */
  Detection();

  /**
   * @brief Construct detection with all parameters
   * 
   * @param bbox Bounding box of the detection
   * @param confidence Confidence score [0.0, 1.0]
   * @param classId Class ID (e.g., 0 for person in COCO dataset)
   * @param className Human-readable class name (e.g., "person")
   */
  Detection(const BoundingBox& bbox,
           float confidence,
           int classId,
           const std::string& className);

  /**
   * @brief Get bounding box
   * @return const BoundingBox& Reference to bounding box
   */
  const BoundingBox& getBoundingBox() const;

  /**
   * @brief Get confidence score
   * @return float Confidence [0.0, 1.0]
   */
  float getConfidence() const;

  /**
   * @brief Get class ID
   * @return int Numeric class identifier
   */
  int getClassId() const;

  /**
   * @brief Get class name
   * @return const std::string& Human-readable class label
   */
  const std::string& getClassName() const;

  /**
   * @brief Set bounding box
   * @param bbox New bounding box
   */
  void setBoundingBox(const BoundingBox& bbox);

  /**
   * @brief Set confidence score
   * @param confidence New confidence value
   */
  void setConfidence(float confidence);

  /**
   * @brief Set class information
   * @param classId Numeric class ID
   * @param className Human-readable class name
   */
  void setClass(int classId, const std::string& className);

  /**
   * @brief Check if detection is valid (confidence above threshold)
   * @param threshold Minimum confidence threshold
   * @return true if confidence >= threshold
   */
  bool isValid(float threshold) const;

 private:
  BoundingBox bbox_;    ///< Bounding box of detected object
  float confidence_;    ///< Detection confidence score [0.0, 1.0]
  int classId_;        ///< Numeric class identifier
  std::string className_;  ///< Human-readable class label
};

}  // namespace detection
}  // namespace perception

