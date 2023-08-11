#include "RobotController.hpp"

RobotController::RobotController(const string& robotIP, const int frequency)
    : rtde_control(robotIP, frequency)
{

}

void RobotController::servoL(vector<double> targetPose, vector<double> currentPose, double time, double lookahead_time, double scalingFactor)
{
    double acceleration = 0.1; //these inputs get ignored in the current version of ur_rtde
    double speed = 0.1;
    double gain = 100; //minimum tolarated value is 100

    VectorXd actualTargetPosition(6);
    actualTargetPosition << targetPose[0], targetPose[1], targetPose[2], targetPose[3], targetPose[4], targetPose[5];
    VectorXd currentPosition(6);
    currentPosition << currentPose[0], currentPose[1], currentPose[2], currentPose[3], currentPose[4], currentPose[5];
    
    VectorXd intermediateTargetPosition = currentPosition + (actualTargetPosition - currentPosition) * scalingFactor;

    vector<double> intermediateTargetPose = {intermediateTargetPosition[0], intermediateTargetPosition[1], intermediateTargetPosition[2],
                                         intermediateTargetPosition[3], intermediateTargetPosition[4], intermediateTargetPosition[5]};

    rtde_control.servoL(intermediateTargetPose, acceleration, speed, time, lookahead_time, gain);
}

void RobotController::speedControlPD(vector<double> targetPose, vector<double> currentPose, vector<double> currentSpeed, double Kp, double Kd, double acceleration)
{
    VectorXd targetPoseVec(6);
    targetPoseVec << targetPose[0], targetPose[1], targetPose[2], targetPose[3], targetPose[4], targetPose[5];
    VectorXd currentPoseVec(6);
    currentPoseVec << currentPose[0], currentPose[1], currentPose[2], currentPose[3], currentPose[4], currentPose[5];
    VectorXd currentSpeedVec(6);
    currentSpeedVec << currentSpeed[0], currentSpeed[1], currentSpeed[2], currentSpeed[3], currentSpeed[4], currentSpeed[5];

    VectorXd desiredSpeedVec = Kp * (targetPoseVec - currentPoseVec) - Kd * currentSpeedVec;

    vector<double> desiredSpeed = {desiredSpeedVec[0], desiredSpeedVec[1], desiredSpeedVec[2],
                                   desiredSpeedVec[3], desiredSpeedVec[4], desiredSpeedVec[5]};
                                   
    rtde_control.speedL(desiredSpeed, acceleration);
}

void RobotController::haltRobot()
{
    //rtde_control.speedStop(1.0);
    rtde_control.servoStop(1.0); //typical 25ms
}
