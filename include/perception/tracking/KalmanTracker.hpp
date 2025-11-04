/**
 * @file KalmanTracker.hpp
 * @brief Kalman Filter-based multi-object tracker
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include <vector>
#include <map>
#include <memory>
#include "ITracker.hpp"
#include "Track.hpp"
#include "KalmanFilter.hpp"
#include "../detection/Detection.hpp"

namespace perception {
namespace tracking {

/**
 * @class KalmanTracker
 * @brief Multi-object tracker using Kalman Filters
 * 
 * Implements the ITracker interface using Kalman filters for state prediction
 * and IoU-based data association. Each track has its own Kalman filter instance.
 * 
 * Data association strategy:
 * 1. Predict all track positions
 * 2. Compute IoU between predicted boxes and detected boxes
 * 3. Use greedy matching or Hungarian algorithm for association
 * 4. Update matched tracks, create new tracks, delete old tracks
 */
class KalmanTracker : public ITracker {
 public:
  /**
   * @brief Construct Kalman tracker with parameters
   * 
   * @param maxAge Maximum frames to keep track without updates
   * @param minHits Minimum hits before confirming track
   * @param iouThreshold IoU threshold for data association
   */
  explicit KalmanTracker(int maxAge = 30,
                        int minHits = 3,
                        float iouThreshold = 0.3f);

  /**
   * @brief Destructor
   */
  ~KalmanTracker() override;

  /**
   * @brief Update tracks with new detections from current frame
   * 
   * Performs prediction, data association, and track management.
   * 
   * @param detections Vector of detections from current frame
   * @param timestamp Current frame timestamp in seconds
   */
  void update(const std::vector<detection::Detection>& detections,
             double timestamp) override;

  /**
   * @brief Get all currently active tracks
   * 
   * @return std::vector<Track> Vector of confirmed active tracks
   */
  std::vector<Track> getActiveTracks() const override;

  /**
   * @brief Get a specific track by ID
   * 
   * @param trackId Unique identifier of the track
   * @return Track Requested track object
   * @throws std::runtime_error if track ID not found
   */
  Track getTrackById(int trackId) const override;

  /**
   * @brief Reset tracker, removing all active tracks
   */
  void reset() override;

  /**
   * @brief Set maximum number of frames a track can be lost before deletion
   * 
   * @param maxAge Maximum age in frames
   */
  void setMaxAge(int maxAge) override;

  /**
   * @brief Get number of currently active tracks
   * 
   * @return int Number of active tracks
   */
  int getTrackCount() const override;

 private:
  /**
   * @brief Predict all track states for current timestamp
   */
  void predictTracks();

  /**
   * @brief Associate detections with existing tracks
   * 
   * Uses IoU between predicted track boxes and detection boxes.
   * 
   * @param detections Current frame detections
   * @return std::vector<std::pair<int, int>> Pairs of (track_idx, detection_idx)
   */
  std::vector<std::pair<int, int>> associateDetectionsToTracks(
      const std::vector<detection::Detection>& detections);

  /**
   * @brief Create new track from unmatched detection
   * 
   * @param detection Detection to initialize track with
   */
  void createNewTrack(const detection::Detection& detection);

  /**
   * @brief Update track with matched detection
   * 
   * @param trackIdx Index of track to update
   * @param detection Matched detection
   */
  void updateTrack(int trackIdx, const detection::Detection& detection);

  /**
   * @brief Handle unmatched tracks (prediction only)
   */
  void handleUnmatchedTracks(const std::vector<int>& unmatchedIndices);

  /**
   * @brief Delete old or lost tracks
   */
  void deleteOldTracks();

  /**
   * @brief Generate unique track ID
   * 
   * @return int New unique track ID
   */
  int generateTrackId();

  std::vector<Track> tracks_;               ///< Active tracks
  std::map<int, KalmanFilter> kalmanFilters_;  ///< Kalman filter per track
  int nextTrackId_;                        ///< Next available track ID
  int maxAge_;                             ///< Max frames without update
  int minHits_;                            ///< Min hits to confirm track
  float iouThreshold_;                     ///< IoU threshold for matching
  double currentTimestamp_;                ///< Current frame timestamp
};

}  // namespace tracking
}  // namespace perception

