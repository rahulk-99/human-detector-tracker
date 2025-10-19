/**
 * @file IDetector.hpp
 * @brief Interface for human detection algorithms
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include <vector>
#include <memory>
#include "Detection.hpp"

namespace perception {
namespace detection {

/**
 * @interface IDetector
 * @brief Abstract interface for human detection algorithms
 * 
 * This interface defines the contract for any human detection algorithm.
 * Implementations could use YOLO, SSD, or other object detection methods.
 * Follows the Strategy design pattern to allow switching between detectors.
 */
class IDetector {
 public:
  /**
   * @brief Virtual destructor for proper cleanup
   */
  virtual ~IDetector() = default;

  /**
   * @brief Detect humans in an image frame
   * 
   * @param frame Input image as raw data (OpenCV Mat format expected)
   * @param width Width of the image in pixels
   * @param height Height of the image in pixels
   * @param channels Number of color channels (3 for RGB/BGR)
   * @return std::vector<Detection> Vector of detected humans with bounding boxes
   */
  virtual std::vector<Detection> detect(const unsigned char* frame,
                                       int width,
                                       int height,
                                       int channels) = 0;

  /**
   * @brief Set confidence threshold for detections
   * 
   * @param threshold Minimum confidence score (0.0 to 1.0) to consider a detection valid
   */
  virtual void setConfidenceThreshold(float threshold) = 0;

  /**
   * @brief Get current confidence threshold
   * 
   * @return float Current confidence threshold value
   */
  virtual float getConfidenceThreshold() const = 0;

  /**
   * @brief Check if detector is initialized and ready
   * 
   * @return true if detector is ready to process frames
   * @return false if initialization failed or not yet initialized
   */
  virtual bool isInitialized() const = 0;
};

}  // namespace detection
}  // namespace perception

