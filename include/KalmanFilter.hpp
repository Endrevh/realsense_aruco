#include <Eigen/Dense>
#include <random>

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

using namespace Eigen;
using namespace std;

class KalmanFilterSintef {
    private:
        double dt_; // Time step
        MatrixXd A_, C_, Q_, R_; // System matrices (state transition, observation, process noise, measurement noise)
        VectorXd x_; // State vector
        MatrixXd P_; // Covariance matrix
        MatrixXd I_; // Identity matrix
        VectorXd randomWalkCoeff_; // Random walk coefficients
        mt19937 randomGenerator;
        uniform_real_distribution<double> randomDistribution;

    public:
        // Constructor that takes system matrices (A, C, Q, R) and initializes the Kalman filter
        KalmanFilterSintef(double dt, const MatrixXd& A, const MatrixXd& C, const MatrixXd& Q, const MatrixXd& R, const VectorXd& randomWalkCoeff);

        // Reset filter 
        void reset(const VectorXd& z);

        // Predict step of the Kalman filter
        void predict();

        // Correction step of the Kalman filter
        void correction(const VectorXd& z);

        // Accessor method to get the current state vector
        const VectorXd& getState();

        double getDt();
};

#endif