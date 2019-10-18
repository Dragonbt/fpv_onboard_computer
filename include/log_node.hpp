#ifndef _LOG_NODE_
#define _LOG_NODE_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <mutex>
#include "clock.hpp"
#include <atomic>

#define WRITE_VIDEO_WAIT_FOR_MS 50

using namespace std;
using namespace cv;

typedef struct{
    bool video = false;
}LogStatus;

extern LogStatus log_status_topic;
extern mutex log_status_mutex;

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