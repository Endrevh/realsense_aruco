#include "KalmanFilter.hpp"


// Constructor that takes system matrices (A, C, Q, R) and initializes the Kalman filter
KalmanFilterSintef::KalmanFilterSintef(double dt, const MatrixXd& A, const MatrixXd& C, const MatrixXd& Q, const MatrixXd& R)
    : dt_(dt), A_(A), C_(C), Q_(Q), R_(R)
{
    x_.resize(A.rows()); // Initialize state vector x with appropriate size
    P_.resize(A.rows(), A.rows()); // Initialize covariance matrix P with appropriate size
    I_.resize(A.rows(), A.rows());
    I_.setIdentity(); // Identity matrix used in the update step
}

void KalmanFilterSintef::reset(const VectorXd& z)
{
    P_.setIdentity();
    x_ << z[0], z[1], z[2], 0, 0, 0;
}

// Predict step of the Kalman filter
void KalmanFilterSintef::predict() {
    x_ = A_ * x_; // Predict the next state using the state transition matrix A, random walk
    P_ = A_ * P_ * A_.transpose() + Q_; // Predict the next covariance matrix using the process noise matrix Q
}

// Update step of the Kalman filter
void KalmanFilterSintef::update(const VectorXd& z) {
    VectorXd y = z - C_ * x_; // Compute the measurement residual
    MatrixXd S = C_ * P_ * C_.transpose() + R_; // Compute the innovation covariance matrix
    MatrixXd K = P_ * C_.transpose() * S.inverse(); // Compute the Kalman gain

    x_ = x_ + K * y; // Update the state using the Kalman gain and the measurement residual

    P_ = (I_ - K * C_) * P_; // Update the covariance matrix using the Kalman gain
}

// Accessor method to get the current state vector
const VectorXd& KalmanFilterSintef::getState() 
{
    return x_;
}
