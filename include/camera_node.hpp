#ifndef _CAMERA_NODE_
#define _CAMERA_NODE_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <mutex>

//#include "detector.hpp"
#include "utils.hpp"

using namespace std;
using namespace cv;

extern high_resolution_clock::time_point init_timepoint;

extern Mat image_topic;
extern mutex image_mutex;

extern int camera_status_topic;
extern mutex camera_status_mutex;

extern int camera_exception_topic;
extern mutex camera_exception_mutex;

extern deque<DetectionResult> target_topic;
extern mutex target_mutex;

void cameraLoop( FileNode camera_config );

void cameraLoopTest();

class Camera{
    public:
        bool init( int id, int _width, int _height );
        bool isOpened( void );
        bool read( Mat& frame );
        void release( void );
    private:
        int width, height;
        double fps, interval;
        bool new_frame;
        VideoCapture cap;
        mutex cap_mutex;
        high_resolution_clock::time_point last_grab;
        void updating( void );
};

class Video{
    public:
        Video( string path, int width, int height );
        void open();
        void close();
        void writeImage(Mat image);
    private:
        string video_path;
        int width, height;
        VideoWriter writer;
};

#endif