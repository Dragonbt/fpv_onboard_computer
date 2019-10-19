#ifndef _LOG_NODE_
#define _LOG_NODE_

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <mutex>
#include <atomic>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "clock.hpp"
#include "struct.hpp"

using namespace std;
using namespace cv;

extern LogStatus log_status_topic;
extern mutex log_status_mutex;

extern vector<PositionNED> position_vec_topic;
extern mutex position_vec_mutex;

extern vector<VelocityNED> velocity_vec_topic;
extern mutex velocity_vec_mutex;

extern vector<EulerAngle> euler_angle_vec_topic;
extern mutex euler_angle_vec_mutex;

extern vector<Input> u_vec_topic;
extern mutex u_vec_mutex;

extern Mat image_topic;
extern mutex image_mutex;

void logLoop( FileNode log_config );

class Video{
    public:
        Video( string path );
        void run( bool flag, Mat frame);
    private:
        atomic<bool> RECORDING = {false};
        VideoWriter video;
        timed_mutex write_mutex;
        int width, height;
        string path;
        void tryWriteFrame( Mat frame );
};

#endif