#include <cmath>
#include <ur_rtde/rtde_receive_interface.h> 

#include "Utility.hpp"
#include "RealSenseCam.hpp"
#include "ObjectTracker.hpp"
#include "RobotController.hpp"
#include "KalmanTesting.hpp"

using namespace aruco;
using namespace ur_rtde;

enum ControllerType {
    SERVO,
    SPEED,
    DEFAULT
};

ControllerType parseControllerType(const std::string& input) {
    if (input == "servo") {
        return SERVO;
    } else if (input == "speed") {
        return SPEED;
    } else {
        return DEFAULT;
    }
}

void controllerNotify(bool& controllerReady, int controller_dt_us, chrono::_V2::system_clock::time_point previousTimestamp) {
    while (true) {
        controllerReady = true;
        this_thread::sleep_for(std::chrono::microseconds(controller_dt_us));
    }
}

void cameraNotify(bool& cameraReady, int camera_dt_us) {
    while (true) {
        cameraReady = true;
        this_thread::sleep_for(std::chrono::microseconds(camera_dt_us));
    }
}

void runVisualServo(ControllerType controllerType, double controllerParameter1, double controllerParameter2)
{
    double scalingFactor;
    double Kp;
    double Kd; 

    if (controllerType == SERVO)
    {
        scalingFactor = controllerParameter1;

        cout << "Selected servo control with scaling factor = " << scalingFactor << endl;
    }
    else if (controllerType == SPEED)
    {
        Kp = controllerParameter1;
        Kd = controllerParameter2;

        cout << "Selected speed control with Kp = " << Kp << " and feedforward coefficient = " << Kd << endl;
    }

    const string cameraMatrixFile = "../camera_matrix.csv";
    const string distortionVectorFile = "../distortion_vector.csv";
    Mat distortionVector = (Mat_<float>(5, 1) << -0.05009072990326561559,0.06612653987891520257,0.0002111387854027448041,0.0004740499866585567101,-0.02313683715979212435);
    //readCameraParameters(cameraMatrixFile, distortionVectorFile, cameraMatrix, distortionVector);
    Mat cameraMatrix = (Mat_<double>(3, 3) <<   387.1227241587743606,0.000000000000000000,324.7014427908744096,
                                                0.000000000000000000, 387.1867402319572875, 242.9256439317197760,
                                                0.000000000000000000, 0.000000000000000000, 1.000000000000000000);

    string robot_ip = "192.168.0.90";
    RTDEReceiveInterface rtdeReceive(robot_ip);
    RobotController robotController(robot_ip, 125);

    if (rtdeReceive.isConnected())
    {
        cout << "Connected to robot on " << robot_ip << endl;
    }

    RealSenseCamera camera;
    camera.start_grabbing();
    rs2::pipeline pipeline = camera.getPipeline();
    pipeline_profile profile = pipeline.get_active_profile();
    rs2::video_stream_profile color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    int cameraFPS = color_stream.fps();
    int width = color_stream.width();
    int height = color_stream.height();
    cout << "Camera is grabbing at " << cameraFPS << " fps, " << width << "x" << height << endl;

    double cameraFrequency = cameraFPS;
    double camera_dt = 1 / cameraFrequency;
    int camera_dt_us = 1000000 * camera_dt;

    double controllerFrequency = 30.0;
    double controller_dt = 1 / controllerFrequency;
    int controller_dt_us = 1000000 * controller_dt;

    MatrixXd A(6, 6);
    A << 1, 0, 0, camera_dt, 0, 0,
         0, 1, 0, 0, camera_dt, 0,
         0, 0, 1, 0, 0, camera_dt,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    MatrixXd C(3, 6);
    C << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0;

    MatrixXd Q(6,6);
    Q.setIdentity();
    Q *= 0.1;

    MatrixXd R(3,3);
    R.setIdentity();
    R *= 0.2;

    VectorXd randomWalkCoeff(6); //not needed, we already have Q (process noise)
    randomWalkCoeff << 0, 0, 0, 0.0, 0.0, 0.0;

    ObjectTracker tracker(camera_dt, A, C, Q, R, randomWalkCoeff);

    //frame transformation flange to camera. XYZ roll-pitch-yaw
    Vector3d tvecCamera_flange(0.07, 0.07, 0.055); //rougly measured
    double roll = M_PI/2;
    double pitch = 3*M_PI/4;
    double yaw = 0;
    Matrix3d R_flange_camera = rollPitchYawToRotationMatrix(roll, pitch, yaw);
    Matrix4d T_flange_camera = buildTransformationMatrix(R_flange_camera, tvecCamera_flange);

    Dictionary aruco_dict = getPredefinedDictionary(cv::aruco::DICT_5X5_100);
    DetectorParameters detectorParams = DetectorParameters();
    cv::aruco::ArucoDetector detector(aruco_dict, detectorParams);

    bool cameraReady = false;
    bool controllerReady = false;

    auto startTime = chrono::high_resolution_clock::now();
    auto previousTimestamp = chrono::high_resolution_clock::now();

    thread cameraThread(cameraNotify, ref(cameraReady), camera_dt_us);
    thread controllerThread(controllerNotify, ref(controllerReady), controller_dt_us, ref(previousTimestamp));

    string fileName = "../data/test.txt";
    ofstream file(fileName);

    file.clear();

    while (true)
    {  
        if (cameraReady)
        {

            cameraReady = false;

            auto start = std::chrono::high_resolution_clock::now();

            Mat frame;
            bool received;
            tie(frame, received) = camera.get_color_data(); // initially typical 20ms, drops to <1ms
            

            auto end = chrono::high_resolution_clock::now();
            double camera_time = chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0;
            cout << "Get color data time:                  " << camera_time << "ms" << endl; 
            Mat grayFrame;
            

            start = std::chrono::high_resolution_clock::now();
            cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY); // typical <1ms
            end = chrono::high_resolution_clock::now();
            double cvt_color_time = chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0;
            cout << "Cvt color time: " << cvt_color_time << "ms" << endl;
            //Mat outputImage = grayFrame.clone();
            
            vector<int> markerIds;
            vector<vector<Point2f>> markerCorners;
            
            start = std::chrono::high_resolution_clock::now();

            detector.detectMarkers(grayFrame, markerCorners, markerIds); // typical 12ms

            end = chrono::high_resolution_clock::now();
            double detect_markers_time = chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0;
            cout << "Detect markers time: " << detect_markers_time << "ms" << endl;

            //detectMarkers(grayFrame, &aruco_dict, markerCorners, markerIds, &detectorParams);

            if (markerIds.size() != 1)
            {
                TrackerState trackerState = tracker.getTrackerState();

                start = std::chrono::high_resolution_clock::now();

                if (trackerState == TrackerState::TRACKING)
                {
                    tracker.skip();
                    robotController.haltRobot();
                }
                
                end = chrono::high_resolution_clock::now();
                double halt_robot_time = chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0;
                cout << "Halt robot time: " << halt_robot_time << "ms" << endl;                
            }
            else
            {
                start = std::chrono::high_resolution_clock::now();

                vector<Vec3d> tvecsObject_camera_temp;
                vector<Vec3d> rvecsObject_camera_temp;

                estimatePoseSingleMarkers(markerCorners, 0.1, cameraMatrix, distortionVector, rvecsObject_camera_temp, tvecsObject_camera_temp);

                Vector3d tvecObject_camera;
                Vector3d rvecObject_camera;
                tvecObject_camera << tvecsObject_camera_temp[0][0], tvecsObject_camera_temp[0][1], tvecsObject_camera_temp[0][2];
                rvecObject_camera << rvecsObject_camera_temp[0][0], rvecsObject_camera_temp[0][1], rvecsObject_camera_temp[0][2];

                //drawFrameAxes(outputImage, cameraMatrix, distortionVector, rvecsObject_camera_temp[0], tvecsObject_camera_temp[0], 0.1);
                
                vector<double> poseFlange_base = rtdeReceive.getActualTCPPose();

                Vector3d tvecFlange_base;
                tvecFlange_base << poseFlange_base[0], poseFlange_base[1], poseFlange_base[2];
                Vector3d rvecFlange_base;
                rvecFlange_base << poseFlange_base[3], poseFlange_base[4], poseFlange_base[5];
                Matrix3d R_base_flange = axisAngleToRotationMatrix(rvecFlange_base);
                Matrix4d T_base_flange = buildTransformationMatrix(R_base_flange, tvecFlange_base);
                
                Matrix4d T_base_camera = T_base_flange * T_flange_camera;
                
                Vector3d tvecObject_base = transformVector(T_base_camera, tvecObject_camera);

                Vector3d tvecCamera_base = T_base_camera.block<3, 1>(0, 3);   
                tracker.update(tvecObject_base, tvecCamera_base);

                end = chrono::high_resolution_clock::now();
                double update_tracker_time = chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0;
                cout << "Update tracker time: " << update_tracker_time << "ms" << endl;
                
            }

            //drawDetectedMarkers(outputImage, markerCorners, markerIds);
            //imshow("Image", outputImage);
            
        }

        if (controllerReady)
        {
            controllerReady = false;

            TrackerState state = tracker.getTrackerState();

            switch (state)
            {
                case TrackerState::SEARCHING:
                    break;
                
                case TrackerState::TRACKING:
                {
                    auto timeStamp = chrono::high_resolution_clock::now();
                    double timeStep = chrono::duration_cast<chrono::microseconds>(timeStamp - previousTimestamp).count() / 1000000.0;
                    previousTimestamp = chrono::high_resolution_clock::now();
                    cout << "Time step: " << timeStep << endl;

                    auto start = std::chrono::high_resolution_clock::now();

                    Vector3d objectPosition_base = tracker.getEstimatedPositionAndVelocity().head<3>();
                    Vector3d objectVelocity_base = tracker.getEstimatedPositionAndVelocity().tail<3>();
                    Vector3d trackingOffset_base = tracker.getTargetOffset();

                    vector<double> poseFlange_base = rtdeReceive.getActualTCPPose();
                    vector<double> internalTCPTarget_base = rtdeReceive.getTargetTCPPose();

                    Vector3d tvecFlange_base(poseFlange_base[0], poseFlange_base[1], poseFlange_base[2]);
                    Vector3d rvecFlange_base(poseFlange_base[3], poseFlange_base[4], poseFlange_base[5]);
                    Matrix3d R_base_flange = axisAngleToRotationMatrix(rvecFlange_base);
                    Matrix4d T_base_flange = buildTransformationMatrix(R_base_flange, tvecFlange_base);

                    Matrix4d T_base_camera = T_base_flange * T_flange_camera;

                    Vector3d tvecCamera_base = T_base_camera.block<3, 1>(0, 3);   
                    Vector3d cameraOffset_base = tvecCamera_base - tvecFlange_base;

                    Vector3d targetPosition_base = objectPosition_base - trackingOffset_base - cameraOffset_base;

                    vector<double> targetPose = {targetPosition_base[0], targetPosition_base[1], targetPosition_base[2],
                                        poseFlange_base[3], poseFlange_base[4], poseFlange_base[5]};

                    vector<double> speedFlange_base = rtdeReceive.getActualTCPSpeed();
                    vector<double> internalTCPSpeed_base = rtdeReceive.getTargetTCPSpeed();

                    vector<double> objectVelocityVector_base = {objectVelocity_base[0], objectVelocity_base[1], objectVelocity_base[2], 0, 0, 0};

                    // write to file on the following format: timestamp, Kalman state estimate 6, trackingOffset 3, tcp_pose 6, internal tcp target 6, min egen target pose 6, tcp speed 6, internal tcp speed 6
                    auto fileTimeStamp = std::chrono::high_resolution_clock::now();
                    double fileTimeStampDouble = std::chrono::duration_cast<std::chrono::microseconds>(fileTimeStamp - startTime).count() / 1000000.0;
                    file << fileTimeStampDouble << "," << objectPosition_base[0] << "," << objectPosition_base[1] << "," << objectPosition_base[2] << "," << objectVelocity_base[0] << "," << objectVelocity_base[1] << "," << objectVelocity_base[2] << "," << trackingOffset_base[0] << "," << trackingOffset_base[1] << "," << trackingOffset_base[2] << "," << poseFlange_base[0] << "," << poseFlange_base[1] << "," << poseFlange_base[2] << "," << poseFlange_base[3] << "," << poseFlange_base[4] << "," << poseFlange_base[5] << "," << internalTCPTarget_base[0] << "," << internalTCPTarget_base[1] << "," << internalTCPTarget_base[2] << "," << internalTCPTarget_base[3] << "," << internalTCPTarget_base[4] << "," << internalTCPTarget_base[5] << "," << targetPose[0] << "," << targetPose[1] << "," << targetPose[2] << "," << targetPose[3] << "," << targetPose[4] << "," << targetPose[5] << "," << speedFlange_base[0] << "," << speedFlange_base[1] << "," << speedFlange_base[2] << "," << speedFlange_base[3] << "," << speedFlange_base[4] << "," << speedFlange_base[5] << "," << internalTCPSpeed_base[0] << "," << internalTCPSpeed_base[1] << "," << internalTCPSpeed_base[2] << "," << internalTCPSpeed_base[3] << "," << internalTCPSpeed_base[4] << "," << internalTCPSpeed_base[5] << endl;

                    auto end = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                    double duration_sec = duration.count() / 1000000.0;
                    cout << "Controller time: " << duration_sec / 1000 << "ms" << endl;

                    double executionTime = controller_dt - duration_sec;
                    double lookahead_time = 0.1;

                    if (controllerType == SERVO)
                    {
                        robotController.servoL(targetPose, poseFlange_base, executionTime, lookahead_time, scalingFactor);
                    }
                    else if (controllerType == SPEED)
                    {
                        robotController.speedControlPD(targetPose, poseFlange_base, objectVelocityVector_base, Kp, controller_dt, Kd);
                    }

                    break;
                }

                default:
                    break;
            }
        }
    }

    camera.close();
    file.close();
}

void runKalmanSimulation()
{
    double cameraFrequency = 30.0;
    double camera_dt = 1 / cameraFrequency;

    MatrixXd A(6, 6);
    A << 1, 0, 0, camera_dt, 0, 0,
         0, 1, 0, 0, camera_dt, 0,
         0, 0, 1, 0, 0, camera_dt,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    MatrixXd C(3, 6);
    C << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0;

    MatrixXd Q(6,6);
    Q << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    MatrixXd R(3,3);
    R.setIdentity();
    R *= 1;

    VectorXd randomWalkCoeff(6); //probably not needed, we already have Q (process noise)
    randomWalkCoeff << 0, 0, 0, 0.0, 0.0, 0.0;
    
    KalmanFilterSintef testFilter(camera_dt, A, C, Q, R, randomWalkCoeff);
    KalmanTesting kalmanTesting(testFilter);
    kalmanTesting.runSimulation(100);
    string fileName = "../data/Q_1_1_R_1.txt";
    kalmanTesting.writeTrajectoriesToFile(fileName);
    cout << "Wrote data to " << fileName << endl;
}

int main(int argc, char** argv)
{
    ControllerType selectedController = DEFAULT;
    double input1 = 0.0;
    double input2 = 0.0;

    if (argc > 1) {
        selectedController = parseControllerType(argv[1]);
    }

    if (selectedController == SPEED) {
        if (argc > 2) {
            std::istringstream iss(argv[2]);
            iss >> input1;
        }
        if (argc > 3) {
            std::istringstream iss(argv[3]);
            iss >> input2;
        }
    } else if (selectedController == SERVO) {
        if (argc > 2) {
            std::istringstream iss(argv[2]);
            iss >> input1;
        }
    }

    // Handle invalid inputs or defaults
    if (selectedController == DEFAULT || input1 < 0.0 || (selectedController == SERVO && input1 > 1.0) || input2 < 0.0 ) {
        selectedController = SERVO;
        input1 = 0.5; // Default value for servo
        input2 = 0.0; // Not used for servo
    }

    runVisualServo(selectedController, input1, input2);

    //runKalmanSimulation();
}