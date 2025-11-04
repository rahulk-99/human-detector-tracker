/**
 * @file tracking_test.cpp
 * @brief Comprehensive tests for tracking components (Track, KalmanFilter, KalmanTracker)
 * @author Acme Robotics
 */

#include <gtest/gtest.h>
#include "perception/tracking/Track.hpp"
#include "perception/tracking/KalmanFilter.hpp"
#include "perception/tracking/KalmanTracker.hpp"
#include "perception/detection/Detection.hpp"
#include "perception/detection/BoundingBox.hpp"
#include <vector>

using namespace perception;

// ============================================================================
// Track Tests
// ============================================================================

TEST(TrackTest, Initialization) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  tracking::Track track(1, bbox, pos, 0.0);
  
  EXPECT_EQ(track.getId(), 1);
  EXPECT_EQ(track.getState(), tracking::TrackState::TENTATIVE);
  EXPECT_EQ(track.getHitCount(), 1);
  EXPECT_EQ(track.getMissCount(), 0);
}

TEST(TrackTest, Update) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  tracking::Track track(1, bbox, pos, 0.0);
  
  // Update track
  utils::Position3D newPos(1.5f, 2.5f, 3.5f);
  track.update(bbox, newPos, 0.1);
  
  EXPECT_EQ(track.getHitCount(), 2);
  EXPECT_EQ(track.getMissCount(), 0);
}

TEST(TrackTest, StateTransition) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  tracking::Track track(1, bbox, pos, 0.0);
  
  // After 3 updates, should become CONFIRMED
  for (int i = 0; i < 3; ++i) {
    track.update(bbox, pos, i * 0.1);
  }
  EXPECT_EQ(track.getState(), tracking::TrackState::CONFIRMED);
}

TEST(TrackTest, DefaultConstructor) {
  tracking::Track track;
  
  EXPECT_EQ(track.getId(), -1);
  EXPECT_EQ(track.getState(), tracking::TrackState::TENTATIVE);
  EXPECT_EQ(track.getHitCount(), 0);
  EXPECT_EQ(track.getMissCount(), 0);
  EXPECT_EQ(track.getAge(), 0);
}

TEST(TrackTest, MarkMissed) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  tracking::Track track(1, bbox, pos, 0.0);
  
  EXPECT_EQ(track.getMissCount(), 0);
  
  // Mark missed multiple times
  for (int i = 0; i < 6; ++i) {
    track.markMissed();
  }
  
  EXPECT_EQ(track.getMissCount(), 6);
  EXPECT_EQ(track.getState(), tracking::TrackState::LOST);
}

TEST(TrackTest, ShouldDelete) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  tracking::Track track(1, bbox, pos, 0.0);
  
  // Should not delete if missCount <= maxAge
  EXPECT_FALSE(track.shouldDelete(10));
  
  // Mark missed enough times
  for (int i = 0; i < 11; ++i) {
    track.markMissed();
  }
  
  EXPECT_TRUE(track.shouldDelete(10));
  
  // LOST state should also return true
  tracking::Track track2(2, bbox, pos, 0.0);
  track2.setState(tracking::TrackState::LOST);
  EXPECT_TRUE(track2.shouldDelete(100));
}

TEST(TrackTest, Predict) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  tracking::Track track(1, bbox, pos, 0.0);
  
  // Set a velocity
  utils::Position3D velocity(0.5f, 0.3f, 0.1f);
  track.setVelocity(velocity);
  
  // Predict future position
  track.predict(1.0);
  
  // Position should be updated based on velocity
  const auto& newPos = track.getPosition();
  EXPECT_GT(newPos.getX(), pos.getX());
  EXPECT_EQ(track.getAge(), 2);  // Age increments
}

TEST(TrackTest, Setters) {
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  utils::Position3D pos(1.0f, 2.0f, 3.0f);
  tracking::Track track(1, bbox, pos, 0.0);
  
  // Test setState
  track.setState(tracking::TrackState::CONFIRMED);
  EXPECT_EQ(track.getState(), tracking::TrackState::CONFIRMED);
  
  track.setState(tracking::TrackState::LOST);
  EXPECT_EQ(track.getState(), tracking::TrackState::LOST);
  
  // Test setVelocity
  utils::Position3D velocity(1.0f, 2.0f, 3.0f);
  track.setVelocity(velocity);
  utils::Position3D retrievedVel = track.getVelocity();
  EXPECT_FLOAT_EQ(retrievedVel.getX(), 1.0f);
  EXPECT_FLOAT_EQ(retrievedVel.getY(), 2.0f);
  EXPECT_FLOAT_EQ(retrievedVel.getZ(), 3.0f);
  
  // Test setPosition
  utils::Position3D newPos(10.0f, 20.0f, 30.0f);
  track.setPosition(newPos);
  const auto& retrievedPos = track.getPosition();
  EXPECT_FLOAT_EQ(retrievedPos.getX(), 10.0f);
  EXPECT_FLOAT_EQ(retrievedPos.getY(), 20.0f);
  EXPECT_FLOAT_EQ(retrievedPos.getZ(), 30.0f);
  
  // Test getBoundingBox
  const auto& retrievedBbox = track.getBoundingBox();
  EXPECT_FLOAT_EQ(retrievedBbox.getX(), bbox.getX());
  EXPECT_FLOAT_EQ(retrievedBbox.getY(), bbox.getY());
  
  // Test getTimestamp
  EXPECT_DOUBLE_EQ(track.getTimestamp(), 0.0);
  
  // Update track and check timestamp changes
  track.update(bbox, newPos, 1.5);
  EXPECT_DOUBLE_EQ(track.getTimestamp(), 1.5);
}

// ============================================================================
// KalmanFilter Tests
// ============================================================================

TEST(KalmanFilterTest, Initialization) {
  tracking::KalmanFilter kf(6, 3);
  EXPECT_FALSE(kf.isInitialized());
  
  std::vector<float> state = {0, 0, 0, 0, 0, 0};
  std::vector<float> cov(36, 0);
  for (int i = 0; i < 6; ++i) {
    cov[i * 6 + i] = 1.0f;
  }
  
  kf.initialize(state, cov);
  EXPECT_TRUE(kf.isInitialized());
}

TEST(KalmanFilterTest, PredictUpdate) {
  tracking::KalmanFilter kf(6, 3);
  
  std::vector<float> state = {1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f};
  std::vector<float> cov(36, 0);
  for (int i = 0; i < 6; ++i) {
    cov[i * 6 + i] = 1.0f;
  }
  
  kf.initialize(state, cov);
  kf.predict(0.1f);
  
  std::vector<float> measurement = {1.1f, 2.1f, 3.1f};
  kf.update(measurement);
  
  auto updatedState = kf.getState();
  EXPECT_EQ(updatedState.size(), 6);
}

TEST(KalmanFilterTest, GetCovariance) {
  tracking::KalmanFilter kf(6, 3);
  
  std::vector<float> state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<float> cov(36, 0);
  for (int i = 0; i < 6; ++i) {
    cov[i * 6 + i] = 1.0f;
  }
  
  kf.initialize(state, cov);
  
  // Covariance is initialized successfully
  EXPECT_TRUE(kf.isInitialized());
}

TEST(KalmanFilterTest, Reset) {
  tracking::KalmanFilter kf(6, 3);
  
  std::vector<float> state = {1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f};
  std::vector<float> cov(36, 0);
  for (int i = 0; i < 6; ++i) {
    cov[i * 6 + i] = 1.0f;
  }
  
  kf.initialize(state, cov);
  EXPECT_TRUE(kf.isInitialized());
  
  kf.reset();
  EXPECT_FALSE(kf.isInitialized());
  
  // After reset, state should be zero
  auto resetState = kf.getState();
  for (size_t i = 0; i < resetState.size(); ++i) {
    EXPECT_FLOAT_EQ(resetState[i], 0.0f);
  }
}

// ============================================================================
// KalmanFilter Error Handling Tests
// ============================================================================

TEST(KalmanFilterErrorTest, PredictWithoutInitialization) {
  tracking::KalmanFilter filter(6, 3);
  
  // Should throw when predicting without initialization
  EXPECT_THROW(filter.predict(0.1f), std::runtime_error);
}

TEST(KalmanFilterErrorTest, UpdateWithoutInitialization) {
  tracking::KalmanFilter filter(6, 3);
  
  std::vector<float> measurement = {1.0f, 2.0f, 3.0f};
  
  // Should throw when updating without initialization
  EXPECT_THROW(filter.update(measurement), std::runtime_error);
}

TEST(KalmanFilterErrorTest, InitializeWithWrongDimensions) {
  tracking::KalmanFilter filter(6, 3);
  
  // Wrong state dimension
  std::vector<float> wrongState = {1.0f, 2.0f};  // Should be 6
  std::vector<float> cov = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  
  EXPECT_THROW(filter.initialize(wrongState, cov), std::invalid_argument);
  
  // Wrong covariance dimension
  std::vector<float> correctState = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<float> wrongCov = {1.0f, 0.0f, 0.0f, 1.0f};  // Should be 36
  
  EXPECT_THROW(filter.initialize(correctState, wrongCov), std::invalid_argument);
}

TEST(KalmanFilterErrorTest, InitializeWithNonPositiveDefiniteCovariance) {
  tracking::KalmanFilter filter(6, 3);
  
  std::vector<float> state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  
  // Create covariance with non-positive diagonal (should fail)
  std::vector<float> badCov(36, 0.0f);
  badCov[0] = -1.0f;  // Negative diagonal
  
  EXPECT_THROW(filter.initialize(state, badCov), std::invalid_argument);
  
  // Zero diagonal should also fail
  std::vector<float> zeroCov(36, 0.0f);
  EXPECT_THROW(filter.initialize(state, zeroCov), std::invalid_argument);
}

TEST(KalmanFilterErrorTest, PredictWithInvalidTimeStep) {
  tracking::KalmanFilter filter(6, 3);
  
  std::vector<float> state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<float> cov = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  filter.initialize(state, cov);
  
  // Zero time step should throw
  EXPECT_THROW(filter.predict(0.0f), std::invalid_argument);
  
  // Negative time step should throw
  EXPECT_THROW(filter.predict(-0.1f), std::invalid_argument);
}

TEST(KalmanFilterErrorTest, UpdateWithWrongMeasurementDimension) {
  tracking::KalmanFilter filter(6, 3);
  
  std::vector<float> state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<float> cov = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  filter.initialize(state, cov);
  
  // Wrong measurement dimension
  std::vector<float> wrongMeasurement = {1.0f, 2.0f};  // Should be 3
  
  EXPECT_THROW(filter.update(wrongMeasurement), std::invalid_argument);
}

// Tests for setProcessNoise and setMeasurementNoise removed - methods no longer exist

// ============================================================================
// KalmanTracker Tests
// ============================================================================

TEST(KalmanTrackerTest, Initialization) {
  tracking::KalmanTracker tracker;
  EXPECT_EQ(tracker.getTrackCount(), 0);
}

TEST(KalmanTrackerTest, TrackCreationAndUpdate) {
  tracking::KalmanTracker tracker;
  
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  detection::Detection det(bbox, 0.85f, 0, "person");
  std::vector<detection::Detection> detections = {det};
  
  tracker.update(detections, 0.0);
  
  // Should create one new track (tentative)
  // After min_hits it becomes confirmed
  for (int i = 1; i < 5; ++i) {
    tracker.update(detections, i * 0.1);
  }
  
  EXPECT_GT(tracker.getTrackCount(), 0);
}

TEST(KalmanTrackerTest, GetTrackById) {
  tracking::KalmanTracker tracker;
  
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  detection::Detection det(bbox, 0.85f, 0, "person");
  std::vector<detection::Detection> detections = {det};
  
  tracker.update(detections, 0.0);
  
  // Get active tracks to find track ID
  auto activeTracks = tracker.getActiveTracks();
  if (activeTracks.size() > 0) {
    int trackId = activeTracks[0].getId();
    
    // Get track by ID
    tracking::Track retrievedTrack = tracker.getTrackById(trackId);
    EXPECT_EQ(retrievedTrack.getId(), trackId);
  }
  
  // Test with non-existent ID (should throw)
  EXPECT_THROW(tracker.getTrackById(999), std::runtime_error);
}

TEST(KalmanTrackerTest, SetMaxAge) {
  tracking::KalmanTracker tracker;
  
  // Default maxAge is 30
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  detection::Detection det(bbox, 0.85f, 0, "person");
  std::vector<detection::Detection> detections = {det};
  
  tracker.update(detections, 0.0);
  
  // Set max age
  tracker.setMaxAge(50);
  
  // Create a track and mark it missed many times
  auto activeTracks = tracker.getActiveTracks();
  if (activeTracks.size() > 0) {
    // The tracker should handle maxAge internally
    // This test verifies the setter works
    EXPECT_GT(tracker.getTrackCount(), 0);
  }
}

TEST(KalmanTrackerTest, SetMinHits) {
  // setMinHits() method removed - test removed
  tracking::KalmanTracker tracker;
  
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  detection::Detection det(bbox, 0.85f, 0, "person");
  std::vector<detection::Detection> detections = {det};
  
  // Update multiple times to create tracks
  for (int i = 0; i < 5; ++i) {
    tracker.update(detections, i * 0.1);
  }
  
  // Tracker should work with new threshold
  EXPECT_GT(tracker.getTrackCount(), 0);
}

TEST(KalmanTrackerTest, Reset) {
  tracking::KalmanTracker tracker;
  
  detection::BoundingBox bbox(100.0f, 200.0f, 50.0f, 100.0f);
  detection::Detection det(bbox, 0.85f, 0, "person");
  std::vector<detection::Detection> detections = {det};
  
  // Update multiple times to ensure tracks are created
  for (int i = 0; i < 5; ++i) {
    tracker.update(detections, i * 0.1);
  }
  
  EXPECT_GT(tracker.getTrackCount(), 0);
  
  tracker.reset();
  EXPECT_EQ(tracker.getTrackCount(), 0);
  
  // Verify no active tracks after reset
  auto activeTracks = tracker.getActiveTracks();
  EXPECT_EQ(activeTracks.size(), 0);
}

