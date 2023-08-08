#include "ObjectTracker.hpp"

ObjectTracker::ObjectTracker(double dt, const MatrixXd& A, const MatrixXd& C, const MatrixXd& Q, const MatrixXd& R, const VectorXd& randomWalkCoeff = VectorXd::Zero(6))
    : kalmanFilter(dt, A, C, Q, R, randomWalkCoeff)
{
    state = TrackerState::SEARCHING;
}

void ObjectTracker::skip()
{
    state = TrackerState::SEARCHING;
}


void ObjectTracker::update(Vector3d objectPosition_base, Vector3d cameraPosition_base)
{
    switch (state)
    {
        case TrackerState::SEARCHING:
            state = TrackerState::TRACKING;

            kalmanFilter.reset(objectPosition_base);

            targetOffset_base = objectPosition_base - cameraPosition_base;

            break;
        
        case TrackerState::TRACKING:
            kalmanFilter.predict();
            kalmanFilter.correction(objectPosition_base);
            break;

        default:
        break;
    }
}

VectorXd ObjectTracker::getEstimatedPositionAndVelocity()
{
    return kalmanFilter.getState();
}

Vector3d ObjectTracker::getTargetOffset()
{
    return targetOffset_base;
}

TrackerState ObjectTracker::getTrackerState()
{
    return state;
}