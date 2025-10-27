/**
 * @file KalmanTracker.cpp
 * @brief Implementation of KalmanTracker class
 * @author Acme Robotics
 * 
 * Phase 1: Enhanced implementation with proper KalmanFilter integration
 */

#include "perception/tracking/KalmanTracker.hpp"
#include <algorithm>
#include <stdexcept>
#include <iostream>

namespace perception {
namespace tracking {

KalmanTracker::KalmanTracker(int maxAge, int minHits, float iouThreshold)
    : nextTrackId_(0),
      maxAge_(maxAge),
      minHits_(minHits),
      iouThreshold_(iouThreshold),
      currentTimestamp_(0.0) {}

KalmanTracker::~KalmanTracker() = default;

void KalmanTracker::update(
    const std::vector<detection::Detection>& detections,
    double timestamp) {
  currentTimestamp_ = timestamp;

  // Step 1: Predict all existing tracks
  predictTracks();

  // Step 2: Associate detections with tracks
  std::vector<std::pair<int, int>> matches =
      associateDetectionsToTracks(detections);

  // Step 3: Update matched tracks
  std::vector<bool> matchedDetections(detections.size(), false);
  std::vector<bool> matchedTracks(tracks_.size(), false);

  for (const auto& match : matches) {
    int trackIdx = match.first;
    int detIdx = match.second;
    updateTrack(trackIdx, detections[detIdx]);
    matchedDetections[detIdx] = true;
    matchedTracks[trackIdx] = true;
  }

  // Step 4: Create new tracks for unmatched detections
  for (size_t i = 0; i < detections.size(); ++i) {
    if (!matchedDetections[i]) {
      createNewTrack(detections[i]);
    }
  }

  // Step 5: Handle unmatched tracks
  std::vector<int> unmatchedIndices;
  for (size_t i = 0; i < matchedTracks.size(); ++i) {
    if (!matchedTracks[i]) {
      unmatchedIndices.push_back(i);
    }
  }
  handleUnmatchedTracks(unmatchedIndices);

  // Step 6: Delete old tracks
  deleteOldTracks();
}

std::vector<Track> KalmanTracker::getActiveTracks() const {
  std::vector<Track> activeTracks;
  for (const auto& track : tracks_) {
    if (track.getState() == TrackState::CONFIRMED) {
      activeTracks.push_back(track);
    }
  }
  return activeTracks;
}

Track KalmanTracker::getTrackById(int trackId) const {
  for (const auto& track : tracks_) {
    if (track.getId() == trackId) {
      return track;
    }
  }
  throw std::runtime_error("Track ID not found");
}

void KalmanTracker::reset() {
  tracks_.clear();
  kalmanFilters_.clear();
  nextTrackId_ = 0;
}

void KalmanTracker::setMaxAge(int maxAge) {
  maxAge_ = maxAge;
}

int KalmanTracker::getTrackCount() const {
  int count = 0;
  for (const auto& track : tracks_) {
    if (track.getState() == TrackState::CONFIRMED) {
      count++;
    }
  }
  return count;
}

void KalmanTracker::setMinHits(int minHits) {
  minHits_ = minHits;
}

void KalmanTracker::setIouThreshold(float threshold) {
  iouThreshold_ = threshold;
}

void KalmanTracker::predictTracks() {
  for (auto& track : tracks_) {
    int trackId = track.getId();
    
    // Use KalmanFilter for prediction if available
    if (kalmanFilters_.find(trackId) != kalmanFilters_.end()) {
      double dt = currentTimestamp_ - track.getTimestamp();
      if (dt > 0.0) {
        try {
          kalmanFilters_[trackId].predict(static_cast<float>(dt));
          
          // Update track with KalmanFilter prediction
          auto kalmanState = kalmanFilters_[trackId].getState();
          if (kalmanState.size() >= 6) {
            utils::Position3D predictedPos(kalmanState[0], kalmanState[1], kalmanState[2]);
            utils::Position3D predictedVel(kalmanState[3], kalmanState[4], kalmanState[5]);
            
            // Update track with predicted state
            track.setPosition(predictedPos);
            track.setVelocity(predictedVel);
            track.predict(currentTimestamp_);
          }
        } catch (const std::exception& e) {
          // Fallback to simple prediction if KalmanFilter fails
          track.predict(currentTimestamp_);
        }
      }
    } else {
      // Fallback to simple prediction if no KalmanFilter
      track.predict(currentTimestamp_);
    }
  }
}

std::vector<std::pair<int, int>> KalmanTracker::associateDetectionsToTracks(
    const std::vector<detection::Detection>& detections) {
  std::vector<std::pair<int, int>> matches;

  if (tracks_.empty() || detections.empty()) {
    return matches;
  }

  // Phase 0: Simple greedy IoU matching
  for (size_t t = 0; t < tracks_.size(); ++t) {
    float maxIou = iouThreshold_;
    int bestDetection = -1;

    for (size_t d = 0; d < detections.size(); ++d) {
      float iou = tracks_[t].getBoundingBox().computeIoU(
          detections[d].getBoundingBox());
      
      if (iou > maxIou) {
        maxIou = iou;
        bestDetection = d;
      }
    }

    if (bestDetection >= 0) {
      matches.push_back({t, bestDetection});
    }
  }

  return matches;
}

void KalmanTracker::createNewTrack(const detection::Detection& detection) {
  int trackId = generateTrackId();
  
  // Estimate position from bounding box center (mock depth)
  const auto& bbox = detection.getBoundingBox();
  utils::Position3D position(bbox.getX() / 100.0f, bbox.getY() / 100.0f, 5.0f);

  Track newTrack(trackId, bbox, position, currentTimestamp_);
  tracks_.push_back(newTrack);

  // Initialize Kalman filter for this track with proper covariance
  KalmanFilter kf(6, 3);
  std::vector<float> initialState = {
      position.getX(), position.getY(), position.getZ(),
      0.0f, 0.0f, 0.0f  // zero initial velocity
  };
  
  // Initialize covariance matrix with reasonable uncertainties
  std::vector<float> initialCov(36, 0.0f);
  // Position uncertainty (diagonal elements)
  initialCov[0 * 6 + 0] = 1.0f;   // x position uncertainty
  initialCov[1 * 6 + 1] = 1.0f;   // y position uncertainty  
  initialCov[2 * 6 + 2] = 1.0f;   // z position uncertainty
  // Velocity uncertainty (higher since we start with zero velocity)
  initialCov[3 * 6 + 3] = 10.0f;  // x velocity uncertainty
  initialCov[4 * 6 + 4] = 10.0f;  // y velocity uncertainty
  initialCov[5 * 6 + 5] = 10.0f;  // z velocity uncertainty
  
  try {
    kf.initialize(initialState, initialCov);
    kalmanFilters_[trackId] = kf;
  } catch (const std::exception& e) {
    std::cerr << "Failed to initialize KalmanFilter for track " << trackId 
              << ": " << e.what() << std::endl;
    // Continue without KalmanFilter - track will use simple prediction
  }
}

void KalmanTracker::updateTrack(int trackIdx,
                               const detection::Detection& detection) {
  const auto& bbox = detection.getBoundingBox();
  utils::Position3D position(bbox.getX() / 100.0f, bbox.getY() / 100.0f, 5.0f);

  // Update track with detection
  tracks_[trackIdx].update(bbox, position, currentTimestamp_);

  // Update Kalman filter with measurement
  int trackId = tracks_[trackIdx].getId();
  if (kalmanFilters_.find(trackId) != kalmanFilters_.end()) {
    try {
      std::vector<float> measurement = {
          position.getX(), position.getY(), position.getZ()
      };
      kalmanFilters_[trackId].update(measurement);
      
      // Update track with KalmanFilter's refined state estimate
      auto kalmanState = kalmanFilters_[trackId].getState();
      if (kalmanState.size() >= 6) {
        utils::Position3D refinedPos(kalmanState[0], kalmanState[1], kalmanState[2]);
        utils::Position3D refinedVel(kalmanState[3], kalmanState[4], kalmanState[5]);
        
        // Update track with KalmanFilter's state estimate
        tracks_[trackIdx].setPosition(refinedPos);
        tracks_[trackIdx].setVelocity(refinedVel);
      }
    } catch (const std::exception& e) {
      // Log error but continue with track update
      std::cerr << "KalmanFilter update failed for track " << trackId 
                << ": " << e.what() << std::endl;
    }
  }
}

void KalmanTracker::handleUnmatchedTracks(
    const std::vector<int>& unmatchedIndices) {
  for (int idx : unmatchedIndices) {
    tracks_[idx].markMissed();
  }
}

void KalmanTracker::deleteOldTracks() {
  auto it = tracks_.begin();
  while (it != tracks_.end()) {
    if (it->shouldDelete(maxAge_)) {
      int trackId = it->getId();
      kalmanFilters_.erase(trackId);
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
}

int KalmanTracker::generateTrackId() {
  return nextTrackId_++;
}

}  // namespace tracking
}  // namespace perception

