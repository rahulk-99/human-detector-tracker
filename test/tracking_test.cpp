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

TEST(KalmanFilterErrorTest, SetProcessNoiseWithWrongDimensions) {
  tracking::KalmanFilter filter(6, 3);
  
  std::vector<float> state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<float> cov = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  filter.initialize(state, cov);
  
  // Wrong dimension for process noise (should be 36)
  std::vector<float> wrongQ = {1.0f, 0.0f, 0.0f, 1.0f};
  
  EXPECT_THROW(filter.setProcessNoise(wrongQ), std::invalid_argument);
}

TEST(KalmanFilterErrorTest, SetMeasurementNoiseWithWrongDimensions) {
  tracking::KalmanFilter filter(6, 3);
  
  std::vector<float> state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<float> cov = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  filter.initialize(state, cov);
  
  // Wrong dimension for measurement noise (should be 9)
  std::vector<float> wrongR = {1.0f, 0.0f, 0.0f, 1.0f};
  
  EXPECT_THROW(filter.setMeasurementNoise(wrongR), std::invalid_argument);
}

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

