#include "detector.hpp"

void LowPassFilter::init(float _scale_cut_freq, float _center_cut_freq)
{
    initialized = false;
    scale_cut_freq = _scale_cut_freq;
    center_cut_freq = _center_cut_freq;
    return;
}

void LowPassFilter::run(Rect2f rect, Rect2f& rect_filtered, high_resolution_clock::time_point now)
{
    int64_t T_ms;
    float T, tau_scale, tau_center;
    Vec2f center = {rect.x + rect.width / 2, rect.y + rect.height / 2};
    float scale = rect.width;
    if( ! initialized )
    {
        prev_time = now;
        prev_center = center;
        prev_scale = scale;
        initialized = true;
        rect_filtered = rect;
        return;
    }
    T_ms = intervalMs(now, prev_time);
    if( T_ms > 200)
    {
        tau_scale = 0;
        tau_center = 0;
        T = 1;
    }
    else{
        tau_scale = 1 / (2 * 3.14 * scale_cut_freq);
        tau_center = 1 / (2 * 3.14 * center_cut_freq);
        T = T_ms / 1000.0;
    }
    center = (tau_center * prev_center + T * center) / (T + tau_center);
    scale = (tau_scale * prev_scale + T * scale) / (T + tau_scale);
    prev_time = now;
    prev_center = center;
    prev_scale = scale;

    rect_filtered.x = center[0] - scale / 2;
    rect_filtered.y = center[1] - scale / 2;
    rect_filtered.width = scale;
    rect_filtered.height = scale;
    return;
}

CircleDetector::CircleDetector(DETECTOR_TYPE type, COLOR _color, float radius, float scale_cut_freq, float center_cut_freq)
{
    this->type = type;
    if(type == PURE_DETECT)
    {
        color = _color;
        if ( radius < 0){
            status = DETECT_TEST;
        }
        else if ( radius == 0){
            status = GLOBAL_DETECT;
        }
        else{
            prev_rec = Rect2f(0, 0, radius * 2, radius * 2);
            status = GLOBAL_ESTIMATE;
        }
    }
    else if(type == DETECT_AND_TRACK)
    {
        color = _color;
        if ( radius < 0){
            status = DETECT_TEST;
        }
        else if ( radius == 0){
            status = GLOBAL_DETECT;
        }
        else{
            prev_rec = Rect2f(0, 0, radius * 2, radius * 2);
            status = GLOBAL_ESTIMATE;
        }
    }
    /*
    else if(type == PURE_TRACK)
    {
        cout << "here" << endl;
        FileStorage config("../config/annotation.yaml", FileStorage::READ);
        string image_path;
        Rect2f roi;
        config["IMAGE"] >> image_path;
        config["ROI"] >> roi;
        cout << image_path << endl;
        cout << roi << endl;
    }
    */
    filter.init(scale_cut_freq, center_cut_freq);
    return;
}

void CircleDetector::colorMask(Mat image, Mat& mask)
{
    Mat hsv, mask1, mask2, kernel;
    cvtColor(image, hsv, COLOR_BGR2HSV);
    switch (color)
    {
    case RED:
        inRange(hsv, Scalar(0, 63, 0), Scalar(10, 255, 255), mask1);
        inRange(hsv, Scalar(170, 63, 0), Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;
        break;
    case BLUE:
        inRange(hsv, Scalar(100, 63, 0), Scalar(124, 255, 255), mask);
        break;
    case NONE:
        inRange(hsv, Scalar(0, 0, 0), Scalar(180, 255, 255), mask);
        break;
    default:
        mask.zeros(image.rows, image.cols, CV_8UC1); 
        break;
    }
    kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
    morphologyEx(mask, mask, MORPH_DILATE, kernel);
    return;
}

void CircleDetector::edgeDetection(Mat image, Mat& edges, Mat& dx, Mat& dy, float canny_thresh)
{
    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(3, 3), 0, 0);
    Sobel(gray, dx, CV_16S, 1, 0, 3, 1, 0, BORDER_REPLICATE);
    Sobel(gray, dy, CV_16S, 0, 1, 3, 1, 0, BORDER_REPLICATE);
    Canny(dx, dy, edges, max(1.0f, canny_thresh / 2), canny_thresh, false);
    //imshow("edges", edges);
    return;
}

float CircleDetector::circleDetect(Mat edges, Mat dx, Mat dy, Rect2f& rect, float dp, float r_min, float r_max, float accum_thresh, float dist_min)
{
    if ( r_min <= 0){
        r_min = 1;
    }
    if( r_max > 400 )
    {
        r_max = 400;
    }
    if ( r_max <= r_min){
        r_max = r_min;
    }
    float x, y, width, height;
    vector<Vec4f> circles;
    Vec4f result;
    HoughCirclesAlt(edges, dx, dy, circles, HOUGH_GRADIENT, dp,
                dist_min,                                        // change this value to detect circles with different distances to each other
                accum_thresh, r_min, r_max, 1
    );
    if ( circles.size() < 1 )
    {
        return -1;
    }
    result = circles[0];
    x = result[0] - result[2];
    y = result[1] - result[2];
    width = 2 * result[2];
    height = width;
    rect = Rect2d(x, y, width, height);
    return result[3] / result[2];
}

void CircleDetector::param_adapt( float r, float &canny_thresh, float &dp, float &delta_r, float &reject_thresh )
{
    if(r < 0)
    {
        canny_thresh = 200;
        dp = 5;
        reject_thresh = 3;
    }
    else if(r  <  100)
    {
        canny_thresh = 150;
        dp = 1;
        reject_thresh = 1.0;
    }
    else{
        canny_thresh = 300;
        dp = 10;
        reject_thresh = 4;
    }
    delta_r = max(r * r / 1600, 10.0f);
    delta_r = min(delta_r, 50.0f);
    return;  
}

bool CircleDetector::run(Mat image, Rect2f &rec, float& confidence)
{
    bool found;
    int width = image.cols;
    int height = image.rows;

    Rect2f est_rec;
    Mat est_roi;

    float delta_r = 10;
    float padding = 1.5;

    float dp = 10;
    float canny_thresh = 200;

    Mat mask, edges, dx, dy;
    float r;
    Rect rec_i;
    float scale;

    float reject_thresh = 0;
    vector<float> scale_steps = {1.0f};
    switch ( status )
    {
    case GLOBAL_ESTIMATE:
        r = prev_rec.width / 2;
        param_adapt(r, canny_thresh, dp, delta_r, reject_thresh);
        colorMask(image, mask);
        edgeDetection(image, edges, dx, dy, canny_thresh);
        edges = edges & mask;
        confidence = circleDetect( edges, dx, dy, rec, dp, r - delta_r / 2, r + delta_r);
        if ( confidence < reject_thresh ){
            fail_cnt ++;
            if ( fail_cnt > fail_tolerent ){
                status = GLOBAL_DETECT;
                cout << "GLOBAL_DETECT" << endl;
            }
            found = false;
            break;
        }
        fail_cnt = 0;
        prev_rec = rec;
        filter.run(rec, rec, high_resolution_clock::now());
        if(type == DETECT_AND_TRACK)
        {
            status = TRACKING;
            tracker.init(resizeRect(rec, 1.05), image);
            cout << "TRACKING" << endl;
        }
        else if(type == PURE_DETECT)
        {
            status = LOCAL_ESTIMATE;
            cout << "LOCAL_ESTIMATE" << endl;
        }
        found = true;
        break;
    case LOCAL_ESTIMATE:
        r = prev_rec.width / 2;
        est_rec = resizeRect( prev_rec, padding ) & Rect2f(0, 0, width, height );
        if ( est_rec.empty() ){
            status = GLOBAL_ESTIMATE;
            cout << "GLOBAL_ESTIMATE" << endl;
            found = false;
            break;
        }
        est_roi = image(est_rec);
        param_adapt(r, canny_thresh, dp, delta_r, reject_thresh);
        colorMask(est_roi, mask);
        edgeDetection(est_roi, edges, dx, dy, canny_thresh);
        edges = edges & mask;
        confidence = circleDetect( edges, dx, dy, rec, dp, r - delta_r / 2, r + delta_r);
        if ( confidence < reject_thresh ){
            status = GLOBAL_ESTIMATE;
            cout << "GLOBAL_ESTIMATE" << endl;
            found = false;
            break;
        }
        fail_cnt = 0;
        rec.x = rec.x + est_rec.x;
        rec.y = rec.y + est_rec.y;
        prev_rec = rec;
        filter.run(rec, rec, high_resolution_clock::now());
        if(type == DETECT_AND_TRACK)
        {
            status = TRACKING;
            tracker.init(resizeRect(rec, 1.05), image);
            cout << "TRACKING" << endl;
        }
        found = true;
        break;
    case GLOBAL_DETECT:
        param_adapt(-1, canny_thresh, dp, delta_r, reject_thresh);
        colorMask(image, mask);
        edgeDetection(image, edges, dx, dy, canny_thresh);
        edges = edges & mask;
        confidence = circleDetect( edges, dx, dy, rec, dp, 10, height / 3);
        if( confidence < reject_thresh ){
            found = false;
            break;
        }
        fail_cnt = 0;
        prev_rec = rec;
        filter.run(rec, rec, high_resolution_clock::now());
        if(type == DETECT_AND_TRACK)
        {
            status = TRACKING;
            tracker.init(resizeRect(rec, 1.05), image);
            cout << "TRACKING" << endl;
        }
        else if(type == PURE_DETECT)
        {
            status = LOCAL_ESTIMATE;
            cout << "LOCAL_ESTIMATE" << endl;
        }
        found = true;
        break;
    /*
    case TRACK_INIT:
        rec = selectROI("select roi", image, false, false);
        tracker.init(rec, image);
        status = TRACKING;
        confidence = 0;
        found = true;
        break;
        //colorMask(image, mask);
        //edgeDetection(image, edges, dx, dy, 150);
        //edges = edges & mask;
        //found = circleDetect( edges, dx, dy, rec, confidence, 1, 1, height / 2, 0);
    */
    case TRACKING:
        r = prev_rec.width / 2;
        tracker.detectMultiScale(image, rec_i, confidence, scale, scale_steps);
        est_rec = resizeRect( rec_i, padding ) & Rect2f(0, 0, width, height );
        if ( est_rec.empty() ){
            status = GLOBAL_ESTIMATE;
            cout << "GLOBAL_ESTIMATE" << endl;
            found = false;
            break;
        }
        est_roi = image(est_rec);
        colorMask(est_roi, mask);
        edgeDetection(est_roi, edges, dx, dy, 300);
        edges = edges & mask;
        confidence = circleDetect( edges, dx, dy, rec, 5, r - 5, est_rec.width / 2);
        if ( confidence < 0.5f ){
            if(type == DETECT_AND_TRACK)
            {
                status = LOCAL_ESTIMATE;
                cout << "LOCAL_ESTIMATE" << endl;
            }
            found = false;
            break;
        }
        rec.x = rec.x + est_rec.x;
        rec.y = rec.y + est_rec.y;
        prev_rec = rec;
        scale = scale * rec.width / rec_i.width;
        tracker.updateTracker(image, rec, scale);
        filter.run(rec, rec, high_resolution_clock::now());
        found = true;
        // if(confidence > 0.3)
        // {
        //     fail_cnt = 0;
        //     tracker.updateTracker(image, rec_i, scale);
        //     rec = rec_i;
        //     filter.run(rec, rec, high_resolution_clock::now());
        //     prev_rec = rec;
        //     found = true;
        //     break;
        // }
        // else{
        //     fail_cnt ++;
        //     if( fail_cnt > 5 )
        //     {
        //         if(type == DETECT_AND_TRACK)
        //         {
        //             status = LOCAL_ESTIMATE;
        //             cout << "LOCAL_ESTIMATE" << endl;
        //         }
        //     }
        // }
        //found = false;
        /*
        rec = tracker.update(image);
        found = true;
        */
        break;
    default:
        break;
    }
    return found;
}

Rect2f CircleDetector::resizeRect( Rect2f rect, float ratio)
{
    float width = rect.width * ratio;
    float height = rect.height * ratio;
    float x = rect.x + rect.width / 2.0 - width / 2.0;
    float y = rect.y + rect.height / 2.0 - height / 2.0;
    Rect2f rect_resized = Rect2f( x, y, width, height );
    return rect_resized;
}

void solvePosition(Rect2f rect, float& distance, Vec3f& orientation, Mat camera_matrix, Mat distort_coeff, float real_scale)
{
    Vec2f top_left, bottom_right, center;
    float scale, mag;
    float fx = camera_matrix.at<float>(0, 0);
    float fy = camera_matrix.at<float>(1, 1);
    float cx = camera_matrix.at<float>(0, 2);
    float cy = camera_matrix.at<float>(1, 2);
    Mat pts(1, 4, CV_32FC2);
    Mat pts_undist(1, 4, CV_32FC2);
    float x[4] = {rect.x, rect.x, rect.x + rect.width, rect.x + rect.width};
    float y[4] = {rect.y, rect.y + rect.height, rect.y + rect.height, rect.y};
    Mat pts_x(1, 4, CV_32FC1, x);
    Mat pts_y(1, 4, CV_32FC1, y);
    vector<Mat> channels{ pts_x, pts_y };
    merge(channels, pts);
    undistortPoints(pts, pts_undist, camera_matrix, distort_coeff, noArray(), camera_matrix);
    top_left = pts_undist.at<Vec2f>(0, 0);
    bottom_right = pts_undist.at<Vec2f>(0, 2);
    center = (top_left + bottom_right) / 2;
    scale = bottom_right[0] - top_left[0];
    orientation = {(center[0] - cx), (center[1] - cy), (fx + fy) / 2};
    mag = norm(orientation, NORM_L2);
    distance = mag * real_scale / scale;
    orientation = distance * orientation / mag;
    return;
}