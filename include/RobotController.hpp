#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <iostream>

#include <ur_rtde/rtde_control_interface.h> 
#include <Eigen/Dense>

using namespace std;
using namespace ur_rtde;
using namespace Eigen;

class RobotController
{
    private:
        RTDEControlInterface rtde_control;

    public:
        RobotController(const string& robotIP, const int frequency);

        void servoL(vector<double> targetPose, vector<double> currentPose, double time, double lookahead_time, double scalingFactor);

        void speedControlPD(vector<double> targetPose, vector<double> currentPose, vector<double> currentSpeed, double Kp, double Kd = 1.0, double acceleration = 1.0);
        
        void haltRobot();
};

#endif