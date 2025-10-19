/**
 * @file KalmanTracker.cpp
 * @brief Implementation of KalmanTracker class
 * @author Acme Robotics
 * 
 * Phase 0: Stub implementation with basic tracking logic
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
    track.predict(currentTimestamp_);
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
  
  // Phase 0: Estimate position from bounding box center (mock depth)
  const auto& bbox = detection.getBoundingBox();
  utils::Position3D position(bbox.getX() / 100.0f, bbox.getY() / 100.0f, 5.0f);

  Track newTrack(trackId, bbox, position, currentTimestamp_);
  tracks_.push_back(newTrack);

  // Initialize Kalman filter for this track
  KalmanFilter kf(6, 3);
  std::vector<float> initialState = {
      position.getX(), position.getY(), position.getZ(),
      0.0f, 0.0f, 0.0f  // zero initial velocity
  };
  std::vector<float> initialCov(36, 0.0f);
  for (int i = 0; i < 6; ++i) {
    initialCov[i * 6 + i] = 1.0f;
  }
  kf.initialize(initialState, initialCov);
  kalmanFilters_[trackId] = kf;
}

void KalmanTracker::updateTrack(int trackIdx,
                               const detection::Detection& detection) {
  const auto& bbox = detection.getBoundingBox();
  utils::Position3D position(bbox.getX() / 100.0f, bbox.getY() / 100.0f, 5.0f);

  tracks_[trackIdx].update(bbox, position, currentTimestamp_);

  // Update Kalman filter
  int trackId = tracks_[trackIdx].getId();
  if (kalmanFilters_.find(trackId) != kalmanFilters_.end()) {
    std::vector<float> measurement = {
        position.getX(), position.getY(), position.getZ()
    };
    kalmanFilters_[trackId].update(measurement);
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

