#ifndef _CAMERA_NODE_
#define _CAMERA_NODE_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "clock.hpp"
#include <thread>
#include <mutex>

using namespace std;
using namespace cv;

extern Mat image_topic;
extern mutex image_mutex;

extern int camera_status_topic;
extern mutex camera_status_mutex;

extern int camera_exception_topic;
extern mutex camera_exception_mutex;

void cameraLoop( int id, int width, int height );

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

#endif