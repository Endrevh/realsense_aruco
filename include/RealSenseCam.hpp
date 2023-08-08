#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#ifndef REALSENSE_H
#define REALSENSE_H

using namespace std;
using namespace cv;
using namespace rs2;

class RealSenseCamera {
    private:
        rs2::pipeline pipeline;
        rs2::config config;
        string nickname;
        vector<string> nicknames;
        bool color_grab_flag;

    public:
        RealSenseCamera(const string& nickname = "RealSense_cam");

        void start_grabbing();

        pair<Mat, bool> get_color_data();     

        void close();

        rs2::pipeline getPipeline();
};

#endif