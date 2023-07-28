#include "KalmanFilter.hpp"

#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

using namespace std;

enum class State {
    SEARCHING,
    TRACKING
};

class ObjectTracker
{
    private:
        KalmanFilterSintef kalmanFilter;
        State state;
        Vector3d targetOffset_base;

    public:
        ObjectTracker(double dt, const MatrixXd& A, const MatrixXd& C, const MatrixXd& Q, const MatrixXd& R);

        void skip();

        void update(Vector3d tvec_base, Vector3d cameraPosition_base);

        VectorXd getEstimatedPosition();

        Vector3d getTargetOffset();
};

#endif