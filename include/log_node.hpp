#ifndef _LOG_NODE_
#define _LOG_NODE_

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>
#include <mutex>

#include "utils.hpp"
#include "struct.hpp"

using namespace std;
using namespace cv;

extern LogCommand log_command_topic;
extern mutex log_command_mutex;

extern vector<PositionNED> position_vec_log_topic;
extern mutex position_vec_log_mutex;

extern vector<VelocityNED> velocity_vec_log_topic;
extern mutex velocity_vec_log_mutex;

extern vector<EulerAngle> euler_angle_vec_log_topic;
extern mutex euler_angle_vec_log_mutex;

extern vector<InputVelocityBody> input_velocity_body_vec_log_topic;
extern mutex input_velocity_body_vec_log_mutex;

extern vector<InputAttitude> input_attitude_vec_log_topic;
extern mutex input_attitude_vec_log_mutex;


extern Mat image_topic;
extern mutex image_mutex;

void logLoop( FileNode log_config );

class Log{
    public:
        Log( string path );
        void open();
        void close();
        void writePositionVec();
        void writeVelocityVec();
        void writeAttitudeVec();
        void writeInputVec();
    private:
        string log_path;
        ofstream log_file;
};

class Video{
    public:
        Video( string path, int width, int height );
        void open();
        void close();
        void writeImage();
    private:
        string video_path;
        int width, height;
        VideoWriter writer;
};

#endif