/**
 * @file YOLODetector.hpp
 * @brief YOLO-based human detector implementation
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include "IDetector.hpp"
#include "Detection.hpp"

#ifdef HAVE_OPENCV
namespace cv {
  class Mat;
}
#endif

namespace perception {
namespace detection {

/**
 * @class YOLODetector
 * @brief Implements human detection using YOLOv8 model
 * 
 * This class uses a pre-trained YOLOv8 model to detect humans in images.
 * It implements the IDetector interface and uses OpenCV DNN module for inference.
 * Supports ONNX model format for cross-platform compatibility.
 */
class YOLODetector : public IDetector {
 public:
  /**
   * @brief Construct YOLO detector with model path
   * 
   * @param modelPath Path to YOLO model file (.onnx format)
   * @param confidenceThreshold Minimum confidence for detections [0.0, 1.0]
   * @param nmsThreshold Non-maximum suppression threshold [0.0, 1.0]
   * @param inputSize Input size for YOLO model (default 640x640)
   */
  explicit YOLODetector(const std::string& modelPath,
                       float confidenceThreshold = 0.5f,
                       float nmsThreshold = 0.4f,
                       int inputSize = 640);

  /**
   * @brief Destructor - cleanup resources
   */
  ~YOLODetector() override;

  /**
   * @brief Detect humans in an image frame
   * 
   * Preprocesses the image, runs YOLO inference, and post-processes
   * results to extract human detections (class ID 0 in COCO dataset).
   * 
   * @param frame Input image as raw data (BGR format)
   * @param width Width of the image in pixels
   * @param height Height of the image in pixels
   * @param channels Number of color channels (should be 3)
   * @return std::vector<Detection> Vector of human detections
   */
  std::vector<Detection> detect(const unsigned char* frame,
                               int width,
                               int height,
                               int channels) override;

  /**
   * @brief Set confidence threshold for detections
   * 
   * @param threshold Minimum confidence score [0.0, 1.0]
   */
  void setConfidenceThreshold(float threshold) override;

  /**
   * @brief Get current confidence threshold
   * 
   * @return float Current confidence threshold value
   */
  float getConfidenceThreshold() const override;

  /**
   * @brief Set Non-Maximum Suppression (NMS) threshold
   * 
   * @param threshold NMS IoU threshold [0.0, 1.0]
   */
  void setNmsThreshold(float threshold);

  /**
   * @brief Get current NMS threshold
   * 
   * @return float Current NMS threshold value
   */
  float getNmsThreshold() const;

  /**
   * @brief Check if detector is initialized and ready
   * 
   * @return true if model is loaded and detector is ready
   * @return false if initialization failed
   */
  bool isInitialized() const override;

  /**
   * @brief Get input size used by the model
   * 
   * @return int Input dimension (e.g., 640 for 640x640)
   */
  int getInputSize() const;

 private:
  /**
   * @brief Initialize the YOLO model
   * 
   * @return true if initialization successful
   * @return false if model loading failed
   */
  bool initialize();

  /**
   * @brief Preprocess image for YOLO inference
   * 
   * Resizes image, normalizes pixel values, and converts to blob format.
   * 
   * @param frame Input image data
   * @param width Image width
   * @param height Image height
   * @param channels Number of channels
   */
  void preprocessImage(const unsigned char* frame,
                      int width,
                      int height,
                      int channels);

  /**
   * @brief Post-process YOLO output to extract detections
   * 
   * Applies NMS and filters for human class (class ID 0).
   * 
   * @param originalWidth Original image width (for scaling boxes)
   * @param originalHeight Original image height (for scaling boxes)
   * @return std::vector<Detection> Processed detections
   */
  std::vector<Detection> postprocessOutput(int originalWidth,
                                          int originalHeight);
  
#ifdef HAVE_OPENCV
  /**
   * @brief Post-process YOLO output to extract detections (with outputs parameter)
   * 
   * Applies NMS and filters for human class (class ID 0).
   * Internal method that takes outputs directly to avoid double forward call.
   * 
   * @param outputs Network output tensors from forward pass
   * @param originalWidth Original image width (for scaling boxes)
   * @param originalHeight Original image height (for scaling boxes)
   * @return std::vector<Detection> Processed detections
   */
  std::vector<Detection> postprocessOutput(const std::vector<cv::Mat>& outputs,
                                          int originalWidth,
                                          int originalHeight);
#endif

  /**
   * @brief Apply Non-Maximum Suppression to remove duplicate detections
   * 
   * @param detections Input detections
   * @return std::vector<Detection> Filtered detections after NMS
   */
  std::vector<Detection> applyNMS(const std::vector<Detection>& detections);

  std::string modelPath_;        ///< Path to YOLO model file
  float confidenceThreshold_;    ///< Minimum confidence for detections
  float nmsThreshold_;          ///< NMS IoU threshold
  int inputSize_;               ///< Model input size (e.g., 640)
  bool initialized_;            ///< Initialization status flag

  // OpenCV DNN network handle (opaque pointer for Phase 0)
  class Impl;
  std::unique_ptr<Impl> pImpl_;  ///< Pimpl idiom for implementation details
};

}  // namespace detection
}  // namespace perception

