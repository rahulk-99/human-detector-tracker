/**
 * @file KalmanFilter.hpp
 * @brief Generic Kalman Filter implementation
 * @author Acme Robotics
 * @version 1.0
 * @date 2025
 * @copyright MIT License
 */

#pragma once

#include <vector>

namespace perception {
namespace tracking {

/**
 * @class KalmanFilter
 * @brief Generic Kalman Filter for state estimation
 * 
 * Implements a discrete-time Kalman filter for linear systems.
 * Provides predict and update steps for optimal state estimation
 * with Gaussian noise assumptions.
 * 
 * State vector: [x, y, z, vx, vy, vz]^T (position and velocity in 3D)
 */
class KalmanFilter {
 public:
  /**
   * @brief Construct Kalman Filter with dimensions
   * 
   * @param stateDim Dimension of state vector (default 6: x,y,z,vx,vy,vz)
   * @param measureDim Dimension of measurement vector (default 3: x,y,z)
   */
  explicit KalmanFilter(int stateDim = 6, int measureDim = 3);

  /**
   * @brief Initialize filter with initial state
   * 
   * @param initialState Initial state vector
   * @param initialCovariance Initial state covariance matrix (flattened)
   */
  void initialize(const std::vector<float>& initialState,
                 const std::vector<float>& initialCovariance);

  /**
   * @brief Predict next state using motion model
   * 
   * Performs time update: x_k|k-1 = F * x_k-1|k-1
   * 
   * @param dt Time step (seconds)
   */
  void predict(float dt);

  /**
   * @brief Update state with new measurement
   * 
   * Performs measurement update using Kalman gain.
   * 
   * @param measurement Measurement vector [x, y, z]
   */
  void update(const std::vector<float>& measurement);

  /**
   * @brief Get current state estimate
   * 
   * @return std::vector<float> Current state vector [x, y, z, vx, vy, vz]
   */
  std::vector<float> getState() const;

  /**
   * @brief Reset filter to initial state
   */
  void reset();

  /**
   * @brief Check if filter is initialized
   * 
   * @return true if initialized
   */
  bool isInitialized() const;

 private:
  /**
   * @brief Build state transition matrix F
   * 
   * @param dt Time step
   */
  void buildTransitionMatrix(float dt);

  /**
   * @brief Build measurement matrix H
   */
  void buildMeasurementMatrix();

  int stateDim_;      ///< State dimension
  int measureDim_;    ///< Measurement dimension
  bool initialized_;  ///< Initialization flag

  std::vector<float> state_;        ///< State vector (n x 1)
  std::vector<float> covariance_;   ///< State covariance (n x n)
  std::vector<float> F_;           ///< State transition matrix (n x n)
  std::vector<float> H_;           ///< Measurement matrix (m x n)
  std::vector<float> Q_;           ///< Process noise covariance (n x n)
  std::vector<float> R_;           ///< Measurement noise covariance (m x m)
  std::vector<float> K_;           ///< Kalman gain (n x m)
};

}  // namespace tracking
}  // namespace perception

