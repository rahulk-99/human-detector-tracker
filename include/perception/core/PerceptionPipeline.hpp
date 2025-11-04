/**
 * @file PerceptionPipeline.hpp
 * @brief Main perception pipeline orchestrating detection and tracking
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include <memory>
#include <vector>
#include <string>
#include "../detection/IDetector.hpp"
#include "../detection/Detection.hpp"
#include "../tracking/ITracker.hpp"
#include "ICoordinateTransform.hpp"
#include "CameraModel.hpp"
#include "../tracking/Track.hpp"

namespace perception {
namespace core {

/**
 * @struct PerceptionOutput
 * @brief Output from perception pipeline for one frame
 */
struct PerceptionOutput {
  std::vector<tracking::Track> tracks;      ///< Active tracked humans
  std::vector<detection::Detection> detections;  ///< Raw detections from detector
  double timestamp;                         ///< Frame timestamp (seconds)
  int frameCount;                           ///< Total frames processed
  bool success;                             ///< Processing success flag
};

/**
 * @class PerceptionPipeline
 * @brief Main perception pipeline integrating detection, tracking, and transforms
 * 
 * This is the facade class that orchestrates the entire perception workflow:
 * 1. Capture/receive image frame
 * 2. Run human detection (YOLO)
 * 3. Transform detections to 3D positions
 * 4. Update tracker with detections
 * 5. Output tracked human positions in robot frame
 * 
 * Implements Facade design pattern to simplify subsystem usage.
 */
class PerceptionPipeline {
 public:
  /**
   * @brief Construct perception pipeline
   * 
   * @param detector Human detector instance (YOLO)
   * @param tracker Multi-object tracker instance (Kalman)
   * @param transformer Coordinate transformer instance
   * @param cameraModel Camera parameters
   */
  PerceptionPipeline(
      std::shared_ptr<detection::IDetector> detector,
      std::shared_ptr<tracking::ITracker> tracker,
      std::shared_ptr<ICoordinateTransform> transformer,
      const CameraModel& cameraModel);

  /**
   * @brief Process a single image frame
   * 
   * Runs detection, tracking, and coordinate transformation pipeline.
   * 
   * @param frame Input image as raw data (BGR format)
   * @param width Image width in pixels
   * @param height Image height in pixels
   * @param channels Number of channels (should be 3)
   * @param timestamp Frame timestamp in seconds
   * @return PerceptionOutput Results with tracked humans in robot frame
   */
  PerceptionOutput processFrame(const unsigned char* frame,
                               int width,
                               int height,
                               int channels,
                               double timestamp);

  /**
   * @brief Process video file
   * 
   * Opens video file and processes all frames, returning results for each frame.
   * 
   * @param videoPath Path to video file
   * @return std::vector<PerceptionOutput> Results for all frames
   */
  std::vector<PerceptionOutput> processVideo(const std::string& videoPath);

  /**
   * @brief Reset pipeline state (clears all tracks)
   */
  void reset();

  /**
   * @brief Get detector instance
   * @return std::shared_ptr<detection::IDetector> Detector pointer
   */
  std::shared_ptr<detection::IDetector> getDetector() const;

  /**
   * @brief Get tracker instance
   * @return std::shared_ptr<tracking::ITracker> Tracker pointer
   */
  std::shared_ptr<tracking::ITracker> getTracker() const;

  /**
   * @brief Get transformer instance
   * @return std::shared_ptr<ICoordinateTransform> Transformer pointer
   */
  std::shared_ptr<ICoordinateTransform> getTransformer() const;

  /**
   * @brief Get camera model
   * @return const CameraModel& Camera model reference
   */
  const CameraModel& getCameraModel() const;

  /**
   * @brief Get total number of frames processed
   * @return int Frame count
   */
  int getFrameCount() const;

  /**
   * @brief Enable/disable visualization
   * 
   * @param enable True to show visualization windows
   */
  void setVisualization(bool enable);

  /**
   * @brief Check if visualization is enabled
   * @return bool Visualization status
   */
  bool isVisualizationEnabled() const;

 private:
  /**
   * @brief Visualize detections and tracks on frame
   * 
   * @param frame Input frame
   * @param width Frame width
   * @param height Frame height
   * @param output Perception output to visualize
   */
  void visualizeFrame(const unsigned char* frame,
                     int width,
                     int height,
                     const PerceptionOutput& output);

  std::shared_ptr<detection::IDetector> detector_;      ///< Detector instance
  std::shared_ptr<tracking::ITracker> tracker_;        ///< Tracker instance
  std::shared_ptr<ICoordinateTransform> transformer_;  ///< Transformer instance
  CameraModel cameraModel_;                            ///< Camera parameters
  int frameCount_;                                     ///< Total frames processed
  bool visualizationEnabled_;                          ///< Visualization flag
};

}  // namespace core
}  // namespace perception

