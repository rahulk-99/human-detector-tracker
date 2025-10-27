/**
 * @file KalmanFilter.cpp
 * @brief Implementation of KalmanFilter class
 * @author Acme Robotics
 * 
 * Phase 1: Enhanced implementation with proper Kalman filter algorithm
 */

#include "perception/tracking/KalmanFilter.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace perception {
namespace tracking {

// Helper functions for matrix operations
namespace {
  /**
   * @brief Matrix multiplication: C = A * B
   * @param A First matrix (m x n)
   * @param B Second matrix (n x p)
   * @param C Result matrix (m x p)
   * @param m Rows of A and C
   * @param n Columns of A, rows of B
   * @param p Columns of B and C
   */
  void matrixMultiply(const std::vector<float>& A, const std::vector<float>& B,
                     std::vector<float>& C, int m, int n, int p) {
    C.resize(m * p, 0.0f);
    for (int i = 0; i < m; ++i) {
      for (int j = 0; j < p; ++j) {
        float sum = 0.0f;
        for (int k = 0; k < n; ++k) {
          sum += A[i * n + k] * B[k * p + j];
        }
        C[i * p + j] = sum;
      }
    }
  }

  /**
   * @brief Matrix transpose: B = A^T
   * @param A Input matrix (m x n)
   * @param B Transposed matrix (n x m)
   * @param m Rows of A
   * @param n Columns of A
   */
  void matrixTranspose(const std::vector<float>& A, std::vector<float>& B, int m, int n) {
    B.resize(n * m);
    for (int i = 0; i < m; ++i) {
      for (int j = 0; j < n; ++j) {
        B[j * m + i] = A[i * n + j];
      }
    }
  }

  /**
   * @brief Matrix addition: C = A + B
   * @param A First matrix
   * @param B Second matrix
   * @param C Result matrix
   * @param size Number of elements
   */
  void matrixAdd(const std::vector<float>& A, const std::vector<float>& B,
                std::vector<float>& C, int size) {
    C.resize(size);
    for (int i = 0; i < size; ++i) {
      C[i] = A[i] + B[i];
    }
  }

  /**
   * @brief Matrix subtraction: C = A - B
   * @param A First matrix
   * @param B Second matrix
   * @param C Result matrix
   * @param size Number of elements
   */
  void matrixSubtract(const std::vector<float>& A, const std::vector<float>& B,
                     std::vector<float>& C, int size) {
    C.resize(size);
    for (int i = 0; i < size; ++i) {
      C[i] = A[i] - B[i];
    }
  }

  /**
   * @brief Matrix inversion using Gaussian elimination (for small matrices)
   * @param A Input matrix (n x n)
   * @param B Inverted matrix (n x n)
   * @param n Matrix dimension
   * @return true if successful, false if singular
   */
  bool matrixInverse(const std::vector<float>& A, std::vector<float>& B, int n) {
    B.resize(n * n);
    
    // Initialize B as identity matrix
    std::fill(B.begin(), B.end(), 0.0f);
    for (int i = 0; i < n; ++i) {
      B[i * n + i] = 1.0f;
    }
    
    // Create augmented matrix [A|I]
    std::vector<float> augmented(n * n * 2);
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        augmented[i * n * 2 + j] = A[i * n + j];
        augmented[i * n * 2 + n + j] = B[i * n + j];
      }
    }
    
    // Gaussian elimination
    for (int i = 0; i < n; ++i) {
      // Find pivot
      int maxRow = i;
      for (int k = i + 1; k < n; ++k) {
        if (std::abs(augmented[k * n * 2 + i]) > std::abs(augmented[maxRow * n * 2 + i])) {
          maxRow = k;
        }
      }
      
      // Swap rows
      if (maxRow != i) {
        for (int j = 0; j < n * 2; ++j) {
          std::swap(augmented[i * n * 2 + j], augmented[maxRow * n * 2 + j]);
        }
      }
      
      // Check for singularity
      if (std::abs(augmented[i * n * 2 + i]) < 1e-10f) {
        return false;
      }
      
      // Make diagonal element 1
      float pivot = augmented[i * n * 2 + i];
      for (int j = 0; j < n * 2; ++j) {
        augmented[i * n * 2 + j] /= pivot;
      }
      
      // Eliminate column
      for (int k = 0; k < n; ++k) {
        if (k != i) {
          float factor = augmented[k * n * 2 + i];
          for (int j = 0; j < n * 2; ++j) {
            augmented[k * n * 2 + j] -= factor * augmented[i * n * 2 + j];
          }
        }
      }
    }
    
    // Extract inverse matrix
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        B[i * n + j] = augmented[i * n * 2 + n + j];
      }
    }
    
    return true;
  }
}

KalmanFilter::KalmanFilter(int stateDim, int measureDim)
    : stateDim_(stateDim),
      measureDim_(measureDim),
      initialized_(false) {
  // Initialize matrices with default sizes
  state_.resize(stateDim_, 0.0f);
  covariance_.resize(stateDim_ * stateDim_, 0.0f);
  F_.resize(stateDim_ * stateDim_, 0.0f);
  H_.resize(measureDim_ * stateDim_, 0.0f);
  Q_.resize(stateDim_ * stateDim_, 0.0f);
  R_.resize(measureDim_ * measureDim_, 0.0f);
  K_.resize(stateDim_ * measureDim_, 0.0f);
}

void KalmanFilter::initialize(const std::vector<float>& initialState,
                              const std::vector<float>& initialCovariance) {
  // Validate input dimensions
  if (initialState.size() != static_cast<size_t>(stateDim_)) {
    throw std::invalid_argument("Initial state dimension mismatch");
  }
  
  if (initialCovariance.size() != static_cast<size_t>(stateDim_ * stateDim_)) {
    throw std::invalid_argument("Initial covariance dimension mismatch");
  }

  // Validate covariance matrix is positive definite (check diagonal elements)
  for (int i = 0; i < stateDim_; ++i) {
    if (initialCovariance[i * stateDim_ + i] <= 0.0f) {
      throw std::invalid_argument("Covariance matrix must be positive definite");
    }
  }

  state_ = initialState;
  covariance_ = initialCovariance;

  // Build default matrices
  buildTransitionMatrix(1.0f);
  buildMeasurementMatrix();

  // Initialize process and measurement noise with reasonable defaults
  std::fill(Q_.begin(), Q_.end(), 0.0f);
  std::fill(R_.begin(), R_.end(), 0.0f);
  
  // Set diagonal elements for process noise (velocity uncertainty)
  for (int i = 0; i < stateDim_; ++i) {
    if (i < 3) {  // Position noise
      Q_[i * stateDim_ + i] = 0.01f;
    } else {      // Velocity noise
      Q_[i * stateDim_ + i] = 0.1f;
    }
  }
  
  // Set diagonal elements for measurement noise
  for (int i = 0; i < measureDim_; ++i) {
    R_[i * measureDim_ + i] = 0.1f;
  }

  initialized_ = true;
}

void KalmanFilter::predict(float dt) {
  if (!initialized_) {
    throw std::runtime_error("Kalman filter not initialized");
  }

  if (dt <= 0.0f) {
    throw std::invalid_argument("Time step must be positive");
  }

  // Build transition matrix with current time step
  buildTransitionMatrix(dt);

  // Predict state: x_k|k-1 = F * x_k-1|k-1
  std::vector<float> predictedState(stateDim_);
  for (int i = 0; i < stateDim_; ++i) {
    predictedState[i] = 0.0f;
    for (int j = 0; j < stateDim_; ++j) {
      predictedState[i] += F_[i * stateDim_ + j] * state_[j];
    }
  }
  state_ = predictedState;

  // Predict covariance: P_k|k-1 = F * P_k-1|k-1 * F^T + Q
  std::vector<float> temp1, temp2, F_transpose;
  
  // temp1 = F * P
  matrixMultiply(F_, covariance_, temp1, stateDim_, stateDim_, stateDim_);
  
  // F_transpose = F^T
  matrixTranspose(F_, F_transpose, stateDim_, stateDim_);
  
  // temp2 = temp1 * F^T = F * P * F^T
  matrixMultiply(temp1, F_transpose, temp2, stateDim_, stateDim_, stateDim_);
  
  // covariance_ = temp2 + Q = F * P * F^T + Q
  matrixAdd(temp2, Q_, covariance_, stateDim_ * stateDim_);
}

void KalmanFilter::update(const std::vector<float>& measurement) {
  if (!initialized_) {
    throw std::runtime_error("Kalman filter not initialized");
  }
  
  if (measurement.size() != static_cast<size_t>(measureDim_)) {
    throw std::invalid_argument("Measurement dimension mismatch");
  }

  // Innovation (residual): y = z - H * x_k|k-1
  std::vector<float> innovation(measureDim_);
  for (int i = 0; i < measureDim_; ++i) {
    innovation[i] = measurement[i];
    for (int j = 0; j < stateDim_; ++j) {
      innovation[i] -= H_[i * stateDim_ + j] * state_[j];
    }
  }

  // Innovation covariance: S = H * P_k|k-1 * H^T + R
  std::vector<float> temp1, temp2, H_transpose, S;
  
  // temp1 = H * P
  matrixMultiply(H_, covariance_, temp1, measureDim_, stateDim_, stateDim_);
  
  // H_transpose = H^T
  matrixTranspose(H_, H_transpose, measureDim_, stateDim_);
  
  // temp2 = temp1 * H^T = H * P * H^T
  matrixMultiply(temp1, H_transpose, temp2, measureDim_, stateDim_, measureDim_);
  
  // S = temp2 + R = H * P * H^T + R
  matrixAdd(temp2, R_, S, measureDim_ * measureDim_);

  // Kalman gain: K = P_k|k-1 * H^T * S^-1
  std::vector<float> S_inverse, temp3;
  
  // Check if S is invertible
  if (!matrixInverse(S, S_inverse, measureDim_)) {
    throw std::runtime_error("Innovation covariance matrix is singular");
  }
  
  // temp3 = P * H^T
  matrixMultiply(covariance_, H_transpose, temp3, stateDim_, stateDim_, measureDim_);
  
  // K = temp3 * S^-1 = P * H^T * S^-1
  matrixMultiply(temp3, S_inverse, K_, stateDim_, measureDim_, measureDim_);

  // Update state: x_k|k = x_k|k-1 + K * y
  std::vector<float> correction(stateDim_);
  for (int i = 0; i < stateDim_; ++i) {
    correction[i] = 0.0f;
    for (int j = 0; j < measureDim_; ++j) {
      correction[i] += K_[i * measureDim_ + j] * innovation[j];
    }
    state_[i] += correction[i];
  }

  // Update covariance: P_k|k = (I - K * H) * P_k|k-1
  std::vector<float> I_minus_KH(stateDim_ * stateDim_), temp4;
  
  // Initialize identity matrix
  std::fill(I_minus_KH.begin(), I_minus_KH.end(), 0.0f);
  for (int i = 0; i < stateDim_; ++i) {
    I_minus_KH[i * stateDim_ + i] = 1.0f;
  }
  
  // temp4 = K * H
  matrixMultiply(K_, H_, temp4, stateDim_, measureDim_, stateDim_);
  
  // I_minus_KH = I - K * H
  matrixSubtract(I_minus_KH, temp4, I_minus_KH, stateDim_ * stateDim_);
  
  // covariance_ = I_minus_KH * P = (I - K * H) * P
  matrixMultiply(I_minus_KH, covariance_, temp4, stateDim_, stateDim_, stateDim_);
  covariance_ = temp4;
}

std::vector<float> KalmanFilter::getState() const {
  return state_;
}

std::vector<float> KalmanFilter::getCovariance() const {
  return covariance_;
}

void KalmanFilter::setProcessNoise(const std::vector<float>& Q) {
  if (Q.size() != static_cast<size_t>(stateDim_ * stateDim_)) {
    throw std::invalid_argument("Process noise matrix dimension mismatch");
  }
  
  // Validate Q is positive semi-definite (check diagonal elements)
  for (int i = 0; i < stateDim_; ++i) {
    if (Q[i * stateDim_ + i] < 0.0f) {
      throw std::invalid_argument("Process noise matrix must be positive semi-definite");
    }
  }
  
  Q_ = Q;
}

void KalmanFilter::setMeasurementNoise(const std::vector<float>& R) {
  if (R.size() != static_cast<size_t>(measureDim_ * measureDim_)) {
    throw std::invalid_argument("Measurement noise matrix dimension mismatch");
  }
  
  // Validate R is positive definite (check diagonal elements)
  for (int i = 0; i < measureDim_; ++i) {
    if (R[i * measureDim_ + i] <= 0.0f) {
      throw std::invalid_argument("Measurement noise matrix must be positive definite");
    }
  }
  
  R_ = R;
}

void KalmanFilter::reset() {
  std::fill(state_.begin(), state_.end(), 0.0f);
  std::fill(covariance_.begin(), covariance_.end(), 0.0f);
  initialized_ = false;
}

bool KalmanFilter::isInitialized() const {
  return initialized_;
}

void KalmanFilter::buildTransitionMatrix(float dt) {
  // State transition matrix for constant velocity model
  // For 6D state [x, y, z, vx, vy, vz]:
  // F = [I_3  dt*I_3]
  //     [0    I_3   ]
  
  std::fill(F_.begin(), F_.end(), 0.0f);
  
  // Set diagonal elements to 1
  for (int i = 0; i < stateDim_; ++i) {
    F_[i * stateDim_ + i] = 1.0f;
  }
  
  // Add dt terms for position-velocity coupling
  // This assumes first half of state is position, second half is velocity
  int posDim = stateDim_ / 2;
  if (stateDim_ % 2 == 0 && posDim > 0) {
    for (int i = 0; i < posDim; ++i) {
      F_[i * stateDim_ + (i + posDim)] = dt;
    }
  } else {
    // Fallback for non-standard dimensions - assume first 3 are position
    int minDim = std::min(3, stateDim_ / 2);
    for (int i = 0; i < minDim; ++i) {
      if (i + 3 < stateDim_) {
        F_[i * stateDim_ + (i + 3)] = dt;
      }
    }
  }
}

void KalmanFilter::buildMeasurementMatrix() {
  // Measurement matrix: H = [I_m  0]
  // We only measure the first measureDim_ elements of the state
  std::fill(H_.begin(), H_.end(), 0.0f);
  
  for (int i = 0; i < measureDim_; ++i) {
    if (i < stateDim_) {
      H_[i * stateDim_ + i] = 1.0f;
    }
  }
}

}  // namespace tracking
}  // namespace perception

