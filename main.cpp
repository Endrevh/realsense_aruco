#include <cmath>
#include <ur_rtde/rtde_receive_interface.h> 

#include "Utility.hpp"
#include "RealSenseCam.hpp"
#include "ObjectTracker.hpp"
#include "RobotController.hpp"
#include "KalmanTesting.hpp"

using namespace aruco;
using namespace ur_rtde;

void controllerNotify(bool& controllerReady) {
    controllerReady = true;
}

void cameraNotify(bool& cameraReady) {
    cameraReady = true;
}

void runVisualServo()
{
    const string cameraMatrixFile = "../camera_matrix.csv";
    const string distortionVectorFile = "../distortion_vector.csv";
    Mat distortionVector = (Mat_<float>(5, 1) << -0.05009072990326561559,0.06612653987891520257,0.0002111387854027448041,0.0004740499866585567101,-0.02313683715979212435);
    //readCameraParameters(cameraMatrixFile, distortionVectorFile, cameraMatrix, distortionVector);
    Mat cameraMatrix = (Mat_<double>(3, 3) <<   387.1227241587743606,0.000000000000000000,324.7014427908744096,
                                                0.000000000000000000, 387.1867402319572875, 242.9256439317197760,
                                                0.000000000000000000, 0.000000000000000000, 1.000000000000000000);

    // Print data type and size of cameraMatrix and distortionVector
    //cout << "Camera Matrix: " << cameraMatrix << " | " << cameraMatrix.size() << endl;
    //cout << "Distortion Vector: " << distortionfVector << " | " << distortionVector.size() << endl;

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
    int camera_dt_ms = 1000 * camera_dt;

    double controllerFrequency = 125.0;
    double controller_dt = 1 / controllerFrequency;
    int controller_dt_ms = 1000 * controller_dt;

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

    VectorXd randomWalkCoeff(6); //probably not needed, we already have Q (process noise)
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

    thread controllerTimer([&]() {
        while (true) {
            controllerNotify(controllerReady);
            this_thread::sleep_for(std::chrono::milliseconds(controller_dt_ms));
        }
    });

    thread cameraTimer([&]() {
        while (true) {
            cameraNotify(cameraReady);
            this_thread::sleep_for(std::chrono::milliseconds(camera_dt_ms));
        }
    });


    while (true)
    {  
        if (cameraReady)
        {
            cameraReady = false;

            auto start = std::chrono::high_resolution_clock::now();

            Mat frame;
            bool received;
            tie(frame, received) = camera.get_color_data();

            Mat grayFrame;
            cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
            Mat outputImage = grayFrame.clone();

            vector<int> markerIds;
            vector<vector<Point2f>> markerCorners;
            
            detector.detectMarkers(grayFrame, markerCorners, markerIds);

            //detectMarkers(grayFrame, &aruco_dict, markerCorners, markerIds, &detectorParams);
            
            if (markerIds.size() != 1)
            {
                tracker.skip();
                robotController.haltSpeedControl();
            }
            else
            {
                vector<Vec3d> tvecsObject_camera_temp;
                vector<Vec3d> rvecsObject_camera_temp;

                estimatePoseSingleMarkers(markerCorners, 0.1, cameraMatrix, distortionVector, rvecsObject_camera_temp, tvecsObject_camera_temp);

                Vector3d tvecObject_camera;
                Vector3d rvecObject_camera;
                tvecObject_camera << tvecsObject_camera_temp[0][0], tvecsObject_camera_temp[0][1], tvecsObject_camera_temp[0][2];
                rvecObject_camera << rvecsObject_camera_temp[0][0], rvecsObject_camera_temp[0][1], rvecsObject_camera_temp[0][2];

                //later: bygg T_camera_object, og regn ut T_base_object = T_base_flange * T_flange_camera * T_camera_object, og extract tvec_base og rvec_base derfra
                //Matrix3d R_camera_object = axisAngleToRotationMatrix(rvecObject_camera);
                //Matrix4d T_camera_object = buildTransformationMatrix(R_camera_object, tvecObject_camera);*/

                drawFrameAxes(outputImage, cameraMatrix, distortionVector, rvecsObject_camera_temp[0], tvecsObject_camera_temp[0], 0.1);
                
                vector<double> poseFlange_base = rtdeReceive.getActualTCPPose();
                //cout << poseFlange_base[0] << " " << poseFlange_base[1] << " " << poseFlange_base[2] << endl;
                //cout << poseFlange_base[3] << " " << poseFlange_base[4] << " " << poseFlange_base[5] << endl;

                Vector3d tvecFlange_base;
                tvecFlange_base << poseFlange_base[0], poseFlange_base[1], poseFlange_base[2];
                Vector3d rvecFlange_base;
                rvecFlange_base << poseFlange_base[3], poseFlange_base[4], poseFlange_base[5];
                Matrix3d R_base_flange = axisAngleToRotationMatrix(rvecFlange_base);
                Matrix4d T_base_flange = buildTransformationMatrix(R_base_flange, tvecFlange_base);
                
                Matrix4d T_base_camera = T_base_flange * T_flange_camera;
                
                Vector3d tvecObject_base = transformVector(T_base_camera, tvecObject_camera);
                //Vector3d tvecObject_flange = transformVector(T_flange_camera, tvecObject_camera);

                Vector3d tvecCamera_base = T_base_camera.block<3, 1>(0, 3);   
                tracker.update(tvecObject_base, tvecCamera_base);
            }

            drawDetectedMarkers(outputImage, markerCorners, markerIds);
            imshow("Image", outputImage);

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
                    Vector3d objectPosition_base = tracker.getEstimatedPositionAndVelocity().head<3>();
                    Vector3d objectVelocity_base = tracker.getEstimatedPositionAndVelocity().tail<3>();
                    Vector3d trackingOffset_base = tracker.getTargetOffset();

                    vector<double> poseFlange_base = rtdeReceive.getActualTCPPose();
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

                    cout << "-----------------------" << endl;
                    cout << "Current position: " << poseFlange_base[0] << " " << poseFlange_base[1] << " " << poseFlange_base[2] << endl;
                    cout << "Estimated position: " << objectPosition_base[0] << " " << objectPosition_base[1] << " " << objectPosition_base[2] << endl;
                    cout << "Estimated velocity: " << objectVelocity_base[0] << " " << objectVelocity_base[1] << " " << objectVelocity_base[2] << endl;
                    cout << "Target position: " << targetPosition_base[0] << " " << targetPosition_base[1] << " " << targetPosition_base[2] << endl;

                    double lookahead_time = 0.1;
                    double reductionFactor = 0.4;
                    //robotController.servoL(targetPose, poseFlange_base, dt, lookahead_time, reductionFactor);

                    vector<double> speedFlange_base = rtdeReceive.getActualTCPSpeed();
                    double Kp = 1.0;
                    double Kd = 0.0;
                    robotController.speedControlPD(targetPose, poseFlange_base, speedFlange_base, Kp, Kd);

                    break;
                }
                default:
                    break;
            }
        }

        
        //auto end = std::chrono::high_resolution_clock::now();
        //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        //std::chrono::milliseconds dt_ms = std::chrono::milliseconds(static_cast<long long>(dt*1000));
        //auto sleepTime = dt_ms - duration;
        //this_thread::sleep_for(std::chrono::milliseconds(sleepTime.count()));
        

        if (cv::waitKey(1) == 'q')
            break;
    }

    camera.close();
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
    // set Q as MatrixXd with 0.1 on first three diagonal entries and 1 on last three
    Q << 0.1, 0, 0, 0, 0, 0,
         0, 0.1, 0, 0, 0, 0,
         0, 0, 0.1, 0, 0, 0,
         0, 0, 0, 10, 0, 0,
         0, 0, 0, 0, 10, 0,
         0, 0, 0, 0, 0, 10;
    
    //Q.setIdentity();
    //Q *= 0.1;

    MatrixXd R(3,3);
    R.setIdentity();
    R *= 0.1;

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
    //runVisualServo();

    runKalmanSimulation();
}