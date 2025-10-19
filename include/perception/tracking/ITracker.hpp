/**
 * @file ITracker.hpp
 * @brief Interface for object tracking algorithms
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include <vector>
#include <memory>
#include "../detection/Detection.hpp"
#include "Track.hpp"

namespace perception {
namespace tracking {

/**
 * @interface ITracker
 * @brief Abstract interface for object tracking algorithms
 * 
 * This interface defines the contract for tracking detected objects across frames.
 * Implementations could use Kalman Filter, Particle Filter, or other tracking methods.
 * Follows the Strategy design pattern.
 */
class ITracker {
 public:
  /**
   * @brief Virtual destructor for proper cleanup
   */
  virtual ~ITracker() = default;

  /**
   * @brief Update tracks with new detections from current frame
   * 
   * This method performs data association between existing tracks and new detections,
   * updates matched tracks, creates new tracks for unmatched detections, and removes
   * lost tracks.
   * 
   * @param detections Vector of detections from current frame
   * @param timestamp Current frame timestamp in seconds
   */
  virtual void update(const std::vector<detection::Detection>& detections,
                     double timestamp) = 0;

  /**
   * @brief Get all currently active tracks
   * 
   * @return std::vector<Track> Vector of active tracks with predicted states
   */
  virtual std::vector<Track> getActiveTracks() const = 0;

  /**
   * @brief Get a specific track by ID
   * 
   * @param trackId Unique identifier of the track
   * @return Track Requested track object
   * @throws std::runtime_error if track ID not found
   */
  virtual Track getTrackById(int trackId) const = 0;

  /**
   * @brief Reset tracker, removing all active tracks
   */
  virtual void reset() = 0;

  /**
   * @brief Set maximum number of frames a track can be lost before deletion
   * 
   * @param maxAge Maximum age in frames
   */
  virtual void setMaxAge(int maxAge) = 0;

  /**
   * @brief Get number of currently active tracks
   * 
   * @return int Number of active tracks
   */
  virtual int getTrackCount() const = 0;
};

}  // namespace tracking
}  // namespace perception

