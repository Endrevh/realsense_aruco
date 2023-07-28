#include "ObjectTracker.hpp"

ObjectTracker::ObjectTracker(double dt, const MatrixXd& A, const MatrixXd& C, const MatrixXd& Q, const MatrixXd& R):
    kalmanFilter(dt, A, C, Q, R)
{
    state = State::SEARCHING;
}

void ObjectTracker::skip()
{
    state = State::SEARCHING;
}


void ObjectTracker::update(Vector3d objectPosition_base, Vector3d cameraPosition_base)
{
    switch (state)
    {
        case State::SEARCHING:
            /*if (isnan(tvec_base[0])) break;*/ //NaN, keep searching
            
            state = State::TRACKING;

            kalmanFilter.reset(objectPosition_base);

            targetOffset_base = objectPosition_base - cameraPosition_base;

            break;
        
        case State::TRACKING:
            /*if (isnan(tvec_base[0])) //NaN, go back to searching
            {
                state = State::SEARCHING;
                break;
            };*/
            kalmanFilter.predict();
            kalmanFilter.update(objectPosition_base);
            break;

        default:
        break;
    }
}

VectorXd ObjectTracker::getEstimatedPosition()
{
    return kalmanFilter.getState();
}

Vector3d ObjectTracker::getTargetOffset()
{
    return targetOffset_base;
}