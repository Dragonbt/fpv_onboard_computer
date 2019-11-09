#ifndef _SEND_NODE_
#define _SEND_NODE_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h> 
#include <string.h>
#include <errno.h>

#include <thread>
#include <mutex>

#include "utils.hpp"
#include "struct.hpp"
#include "const.hpp"

using namespace std;
using namespace cv;
using namespace chrono;

extern int fd;
extern mutex fd_mutex;

extern Topic<Mat> image_topic;

extern Topic<DetectionResult> target_topic;

extern Topic<string> string_topic;

extern Topic<PositionNED> position_ned_topic;

extern Topic<VelocityNED> velocity_ned_topic;

extern Topic<VelocityBody> velocity_body_topic;

extern Topic<EulerAngle> attitude_topic;

extern Topic<VehicleStatus> vehicle_status_topic;

extern Topic<InputAttitude> input_attitude_topic;

extern Topic<float> down_reference_topic;

extern Topic<Vector2f> ne_reference_topic;

extern Topic<int16_t> control_status_topic;

extern Topic<Vector2f> pos_err_xy_topic;

void sendLoop( FileNode send_config );
bool sendSocketInit();

void sendMsg( uint8_t msg_type, uint16_t length, void* buffer );
bool compress( Mat image, double resize_k, int quality, vector<uchar>& img_buffer );

void sendHeartBeat( void );
void sendImg( bool gray, double img_msg_resize, int img_msg_quality );
void sendString( void );

template<typename Struct>
void sendStruct(Topic<Struct> topic, int64_t& timestamp, int msg_type);

template<typename Struct>
void sendStruct(Topic<Struct> topic, int64_t& timestamp, int msg_type)
{
    vector<pair<int64_t, Struct>> topic_vector;
    topic.recent(topic_vector, timestamp);
    if( ! topic_vector.empty() )
    {
        sendMsg( msg_type, (uint16_t) ( topic_vector.size() * sizeof(pair<int64_t, Struct>) ), topic_vector.data() );
    }
    topic_vector.clear();
}
#endif