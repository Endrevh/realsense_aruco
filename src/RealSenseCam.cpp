#include "RealSenseCam.hpp"

RealSenseCamera::RealSenseCamera(const string& nickname)
    : pipeline(rs2::pipeline()), config(rs2::config()), nickname(nickname), color_grab_flag(false)
{
    config.enable_stream(rs2_stream::RS2_STREAM_COLOR, 640, 480, rs2_format::RS2_FORMAT_BGR8, 30);
    nicknames.push_back(nickname);
}

void RealSenseCamera::start_grabbing()
{
    pipeline.start(config);
}

std::pair<Mat, bool> RealSenseCamera::get_color_data() 
{
    rs2::frameset all_data = pipeline.wait_for_frames();
    rs2::frame color_frame = all_data.get_color_frame();
    bool color_grab_flag = false;

    if (color_frame) {
        // Convert the frame data to OpenCV-compatible format
        Mat color_data(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        color_grab_flag = true;
        return { color_data, color_grab_flag };
    } 
    else 
    {
        return { Mat(), color_grab_flag };
    }
}       

void RealSenseCamera::close() 
{
    pipeline.stop();
}

rs2::pipeline RealSenseCamera::getPipeline()
{
    return pipeline;
}