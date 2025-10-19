/**
 * @file Track.cpp
 * @brief Implementation of Track class
 * @author Acme Robotics
 */

#include "perception/tracking/Track.hpp"

namespace perception {
namespace tracking {

Track::Track()
    : trackId_(-1),
      state_(TrackState::TENTATIVE),
      hitCount_(0),
      missCount_(0),
      age_(0),
      timestamp_(0.0),
      lastTimestamp_(0.0) {}

Track::Track(int trackId,
            const detection::BoundingBox& initialBbox,
            const utils::Position3D& initialPosition,
            double timestamp)
    : trackId_(trackId),
      state_(TrackState::TENTATIVE),
      bbox_(initialBbox),
      position_(initialPosition),
      velocity_(0.0f, 0.0f, 0.0f),
      hitCount_(1),
      missCount_(0),
      age_(1),
      timestamp_(timestamp),
      lastTimestamp_(timestamp) {}

int Track::getId() const {
  return trackId_;
}

TrackState Track::getState() const {
  return state_;
}

const detection::BoundingBox& Track::getBoundingBox() const {
  return bbox_;
}

const utils::Position3D& Track::getPosition() const {
  return position_;
}

utils::Position3D Track::getVelocity() const {
  return velocity_;
}

int Track::getHitCount() const {
  return hitCount_;
}

int Track::getMissCount() const {
  return missCount_;
}

int Track::getAge() const {
  return age_;
}

double Track::getTimestamp() const {
  return timestamp_;
}

void Track::update(const detection::BoundingBox& bbox,
                  const utils::Position3D& position,
                  double timestamp) {
  // Update velocity estimate
  if (timestamp > lastTimestamp_) {
    float dt = static_cast<float>(timestamp - lastTimestamp_);
    velocity_ = (position - position_) / dt;
  }

  // Update state
  bbox_ = bbox;
  lastTimestamp_ = timestamp_;
  position_ = position;
  timestamp_ = timestamp;
  hitCount_++;
  missCount_ = 0;
  age_++;

  // Update state to CONFIRMED after enough hits
  if (hitCount_ >= 3 && state_ == TrackState::TENTATIVE) {
    state_ = TrackState::CONFIRMED;
  }
}

void Track::predict(double timestamp) {
  // Predict position using constant velocity model
  if (timestamp > timestamp_) {
    float dt = static_cast<float>(timestamp - timestamp_);
    position_ = position_ + velocity_ * dt;
    timestamp_ = timestamp;
  }
  age_++;
}

void Track::markMissed() {
  missCount_++;
  if (missCount_ > 5) {
    state_ = TrackState::LOST;
  }
}

void Track::setState(TrackState state) {
  state_ = state;
}

void Track::setVelocity(const utils::Position3D& velocity) {
  velocity_ = velocity;
}

bool Track::shouldDelete(int maxAge) const {
  return missCount_ > maxAge || state_ == TrackState::LOST;
}

}  // namespace tracking
}  // namespace perception

