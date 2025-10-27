/**
 * @file Track.hpp
 * @brief Track representation for tracked objects
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include <vector>
#include "../detection/BoundingBox.hpp"
#include "../utils/Position3D.hpp"

namespace perception {
namespace tracking {

/**
 * @enum TrackState
 * @brief State of a tracked object
 */
enum class TrackState {
  TENTATIVE,   ///< Newly initialized, needs confirmation
  CONFIRMED,   ///< Confirmed active track
  LOST         ///< Track lost, pending deletion
};

/**
 * @class Track
 * @brief Represents a tracked object across frames
 * 
 * Contains track ID, state estimate, history, and lifecycle information.
 * Used in conjunction with Kalman filter for state estimation.
 */
class Track {
 public:
  /**
   * @brief Default constructor
   */
  Track();

  /**
   * @brief Construct track with initial detection
   * 
   * @param trackId Unique track identifier
   * @param initialBbox Initial bounding box
   * @param initialPosition Initial 3D position in robot frame
   * @param timestamp Creation timestamp
   */
  Track(int trackId,
       const detection::BoundingBox& initialBbox,
       const utils::Position3D& initialPosition,
       double timestamp);

  /**
   * @brief Get track ID
   * @return int Unique track identifier
   */
  int getId() const;

  /**
   * @brief Get current track state
   * @return TrackState Current state (TENTATIVE/CONFIRMED/LOST)
   */
  TrackState getState() const;

  /**
   * @brief Get current bounding box (predicted or updated)
   * @return const detection::BoundingBox& Current bounding box
   */
  const detection::BoundingBox& getBoundingBox() const;

  /**
   * @brief Get current 3D position in robot frame
   * @return const utils::Position3D& Current position estimate
   */
  const utils::Position3D& getPosition() const;

  /**
   * @brief Get velocity estimate in robot frame
   * @return utils::Position3D Velocity vector (m/s)
   */
  utils::Position3D getVelocity() const;

  /**
   * @brief Get number of consecutive frames track has been detected
   * @return int Hit count
   */
  int getHitCount() const;

  /**
   * @brief Get number of consecutive frames track has been lost
   * @return int Miss count
   */
  int getMissCount() const;

  /**
   * @brief Get age of track in frames
   * @return int Total number of frames since creation
   */
  int getAge() const;

  /**
   * @brief Get last update timestamp
   * @return double Timestamp in seconds
   */
  double getTimestamp() const;

  /**
   * @brief Update track with new detection
   * 
   * @param bbox Updated bounding box
   * @param position Updated 3D position
   * @param timestamp Current timestamp
   */
  void update(const detection::BoundingBox& bbox,
             const utils::Position3D& position,
             double timestamp);

  /**
   * @brief Predict track state for next frame (without measurement)
   * 
   * @param timestamp Prediction timestamp
   */
  void predict(double timestamp);

  /**
   * @brief Mark track as missed (no detection associated)
   */
  void markMissed();

  /**
   * @brief Set track state
   * @param state New track state
   */
  void setState(TrackState state);

  /**
   * @brief Set velocity estimate
   * @param velocity Velocity vector
   */
  void setVelocity(const utils::Position3D& velocity);

  /**
   * @brief Set position estimate
   * @param position Position vector
   */
  void setPosition(const utils::Position3D& position);

  /**
   * @brief Check if track should be deleted (too many misses)
   * @param maxAge Maximum allowed age without updates
   * @return true if track should be deleted
   */
  bool shouldDelete(int maxAge) const;

 private:
  int trackId_;                        ///< Unique track identifier
  TrackState state_;                   ///< Current track state
  detection::BoundingBox bbox_;        ///< Current bounding box
  utils::Position3D position_;         ///< Current 3D position (m)
  utils::Position3D velocity_;         ///< Current velocity (m/s)
  int hitCount_;                       ///< Consecutive detection count
  int missCount_;                      ///< Consecutive miss count
  int age_;                           ///< Total frames since creation
  double timestamp_;                   ///< Last update timestamp (s)
  double lastTimestamp_;              ///< Previous update timestamp (s)
};

}  // namespace tracking
}  // namespace perception

