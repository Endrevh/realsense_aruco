#include <Eigen/Dense>

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

using namespace Eigen;

class KalmanFilterSintef {
    private:
        double dt_; // Time step
        MatrixXd A_, C_, Q_, R_; // System matrices (state transition, observation, process noise, measurement noise)
        VectorXd x_; // State vector
        MatrixXd P_; // Covariance matrix
        MatrixXd I_; // Identity matrix

    public:
        // Constructor that takes system matrices (A, C, Q, R) and initializes the Kalman filter
        KalmanFilterSintef(double dt, const MatrixXd& A, const MatrixXd& C, const MatrixXd& Q, const MatrixXd& R);

        // Reset filter 
        void reset(const VectorXd& z);

        // Predict step of the Kalman filter
        void predict();

        // Update step of the Kalman filter
        void update(const VectorXd& z);

        // Accessor method to get the current state vector
        const VectorXd& getState();
};

#endif