#include <cmath>
#include <ur_rtde/rtde_control_interface.h> 
#include <ur_rtde/rtde_receive_interface.h> 

#include "Utility.hpp"
#include "RealSenseCam.hpp"
#include "ObjectTracker.hpp"

using namespace aruco;
using namespace ur_rtde;

int main(int argc, char** argv)
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
    cout << robot_ip << endl;
    //RTDEReceiveInterface rtdeReceive(robot_ip);
    cout << robot_ip << endl;
    RTDEControlInterface rtdeControl("192.168.0.90");
    cout << robot_ip << endl;


    /*if (rtdeReceive.isConnected())
    {
        cout << "Connected to robot on " << robot_ip << endl;
    }*/

    Vec6d vvec_target = {0, 0, 0, 0, 0, 0};

    double frequency = 125.0;
    double dt = 1 / frequency;

    MatrixXd A(6, 6); //dette er vel ikke riktig?
    A << 1, 0, 0, 1, 0, 0,
         0, 1, 0, 0, 1, 0,
         0, 0, 1, 0, 0, 1,
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

    ObjectTracker tracker(dt, A, C, Q, R);

    //frame transformation flange to camera. XYZ roll-pitch-yaw
    double roll = -M_PI/2;
    double pitch = -3*M_PI/4;
    double yaw = 0;
    Vector3d tvecFlange_camera;
    tvecFlange_camera << 0.05, 0.04, 0.055;
    Matrix3d R_flange_camera = rollPitchYawToRotationMatrix(roll, pitch, yaw);
    Matrix4d T_flange_camera = buildTransformationMatrix(R_flange_camera, tvecFlange_camera);

    RealSenseCamera camera;
    camera.start_grabbing();

    Dictionary aruco_dict = getPredefinedDictionary(cv::aruco::DICT_5X5_100);
    DetectorParameters detectorParams = DetectorParameters();
    ArucoDetector detector(aruco_dict, detectorParams);

    while (true)
    {  
        auto start = std::chrono::high_resolution_clock::now();

        Mat frame;
        bool received;

        tie(frame, received) = camera.get_color_data();

        Mat grayFrame;
        cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
        Mat outputImage = grayFrame.clone();


        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners;
        
        //Ptr<Dictionary> aruco_dict = new Dictionary(getPredefinedDictionary(cv::aruco::DICT_5X5_100));
        //detectMarkers(grayFrame, aruco_dict, markerCorners, markerIds);
        detector.detectMarkers(grayFrame, markerCorners, markerIds);
        //delete aruco_dict;
        
        if (markerIds.size() != 1)
        {
            tracker.skip();
        }
        else
        {
            vector<Vec3d> tvecsObject_camera_temp;
            vector<Vec3d> rvecsObject_camera_temp;

            //cout << markerCorners << endl;
            //cout << cameraMatrix << endl;
            //cout << distortionVector << endl;
            estimatePoseSingleMarkers(markerCorners, 0.1, cameraMatrix, distortionVector, rvecsObject_camera_temp, tvecsObject_camera_temp);

            Vector3d tvecObject_camera;
            Vector3d rvecObject_camera;
            tvecObject_camera << tvecsObject_camera_temp[0][0], tvecsObject_camera_temp[0][1], tvecsObject_camera_temp[0][2];
            rvecObject_camera << rvecsObject_camera_temp[0][0], rvecsObject_camera_temp[0][1], rvecsObject_camera_temp[0][2];

            drawFrameAxes(outputImage, cameraMatrix, distortionVector, rvecsObject_camera_temp[0], tvecsObject_camera_temp[0], 0.1);

            //vector<double> poseFlange_base = rtdeControl.getActualToolFlangePose();
            vector<double> poseFlange_base = {0,0,0,0,0,0};
            Vector3d tvecFlange_base;
            tvecFlange_base << poseFlange_base[0], poseFlange_base[1], poseFlange_base[2];
            Vector3d rotationVector;
            rotationVector << poseFlange_base[3], poseFlange_base[4], poseFlange_base[5];
            Matrix3d R_base_flange = axisAngleToRotationMatrix(rotationVector);
            Matrix4d T_base_flange = buildTransformationMatrix(R_base_flange, tvecFlange_base);
            
            Matrix4d T_base_camera = T_base_flange * T_flange_camera;
            

            Vector3d tvecCamera_base = T_base_camera.block<3, 1>(0, 3);
            Vector3d tvecObject_base = transformVector(T_base_camera, tvecObject_camera);

            //for now: regn ut tvec_base = T_base_tool * tvec_camera og bruk den
            //later: bygg T_camera_object, og regn ut T_base_object = T_base_flange * T_flange_camera * T_camera_object, og extract tvec_base og rvec_base derfra
            
            tracker.update(tvecObject_base, tvecCamera_base);

            VectorXd estimatedPosition_base = tracker.getEstimatedPosition();
            Vector3d targetOffset_base = tracker.getTargetOffset();

            Vector3d targetPosition_base = estimatedPosition_base.head<3>() - targetOffset_base;
            vector<double> targetPose = {targetPosition_base[0], targetPosition_base[1], targetPosition_base[2],
                                        1.24, 2.84, 0.05};

            cout << estimatedPosition_base[0] << " " << estimatedPosition_base[1] << " " << estimatedPosition_base[2] << endl;
            double velocity = 0.1;
            double acceleration = 0.1;
            double lookahead_time = 0.1;
            double gain = 100;
            //rtdeControl.servoL(targetPose, velocity, acceleration, dt, lookahead_time, gain);
        }

        drawDetectedMarkers(outputImage, markerCorners, markerIds);
        imshow("Image", outputImage);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::chrono::milliseconds dt_ms = std::chrono::milliseconds(static_cast<long long>(dt*1000));
        auto sleepTime = dt_ms - duration;
        this_thread::sleep_for(std::chrono::milliseconds(sleepTime.count()));

        if (cv::waitKey(1) == 'q')
            break;
    }

    //delete aruco_dict;
    camera.close();
}