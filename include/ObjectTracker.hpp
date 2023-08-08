#include "KalmanFilter.hpp"

#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

using namespace std;

enum class TrackerState {
    SEARCHING,
    TRACKING
};

class ObjectTracker
{
    private:
        KalmanFilterSintef kalmanFilter;
        TrackerState state;
        Vector3d targetOffset_base;

    public:
        ObjectTracker(double dt, const MatrixXd& A, const MatrixXd& C, const MatrixXd& Q, const MatrixXd& R, const VectorXd& randomWalkCoeff);

        void skip();

        void update(Vector3d objectPosition_base, Vector3d cameraPosition_base = Vector3d(0, 0, 0));

        VectorXd getEstimatedPositionAndVelocity();

        Vector3d getTargetOffset();

        TrackerState getTrackerState();
};

#endif