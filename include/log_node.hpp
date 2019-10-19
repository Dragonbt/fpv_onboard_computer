#ifndef _LOG_NODE_
#define _LOG_NODE_

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <mutex>
#include <atomic>

#include "clock.hpp"
#include "struct.hpp"

using namespace std;
using namespace cv;

extern GCCommand command_topic;
extern mutex command_mutex;

extern vector<PositionNED> position_vec_log_topic;
extern mutex position_vec_log_mutex;

extern vector<VelocityNED> velocity_vec_log_topic;
extern mutex velocity_vec_log_mutex;

extern vector<EulerAngle> euler_angle_vec_log_topic;
extern mutex euler_angle_vec_log_mutex;

extern vector<Input> input_vec_log_topic;
extern mutex input_vec_log_mutex;

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

#endif