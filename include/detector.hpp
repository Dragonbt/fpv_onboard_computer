#ifndef _DETECTOR_
#define _DETECTOR_

#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <chrono>

#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp>
#include <algorithm>

#include "hough.hpp"
#include "kcftracker.hpp"
#include "utils.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono;

enum DETECTOR_TYPE{
    PURE_DETECT = 0,
    PURE_TRACK = 1,
    DETECT_AND_TRACK = 2
};


enum COLOR{
    NONE = -1,
    RED = 0,
    BLUE = 1
};

enum DETECTOR_STATUS{
    GLOBAL_DETECT = 0,
    LOCAL_ESTIMATE = 1,
    GLOBAL_ESTIMATE = 2,
    TRACKING = 3,
    DETECT_TEST = 4,
};

void solvePosition(Rect2f rect, float& distance, Vec3f& orientation, Mat camera_matrix, Mat distort_coeff, float real_scale);

class LowPassFilter{
    public:
    void init(float scale_cut_freq, float center_cut_freq);
    void run(Rect2f rect, Rect2f& rect_filtered, high_resolution_clock::time_point now);
    private:
    float scale_cut_freq, center_cut_freq;
    bool initialized = false;
    high_resolution_clock::time_point prev_time;
    Vec2f prev_center; //x, y, scale in pixel
    float prev_scale;
};

class CircleDetector{
    public:
    CircleDetector(DETECTOR_TYPE type, COLOR _color=NONE, float radius=0, float scale_cut_freq=5, float center_cut_freq=50);
    bool run(Mat image, Rect2f &rect, float& confidence);

    private:
    DETECTOR_STATUS status;
    DETECTOR_TYPE type;
    COLOR color;
    Rect2f prev_rec;
    uint8_t fail_cnt;
    uint8_t fail_tolerent = 20;
    KCFTracker tracker;
    LowPassFilter filter;
    Rect2f resizeRect( Rect2f rect, float ratio);
    void colorMask(Mat image, Mat& mask);
    void edgeDetection(Mat image, Mat& edges, Mat& dx, Mat& dy, float canny_thresh);
    void param_adapt( float r, float &canny_thresh, float &dp, float &delta_r, float &reject_thresh );
    float circleDetect(Mat edge, Mat dx, Mat dy, Rect2f& roi, float dp, float r_min, float r_max, float accum_thresh=15.0, float dist_min=5.0);
};
#endif
