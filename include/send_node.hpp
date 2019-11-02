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
#include "protocol.hpp"

using namespace std;
using namespace cv;
using namespace chrono;

extern int fd;
extern mutex fd_mutex;

extern high_resolution_clock::time_point init_timepoint;
extern Mat image_topic;
extern mutex image_mutex;

extern deque<PositionNED> position_topic;
extern mutex position_mutex;

extern deque<PositionBody> position_body_topic;
extern mutex position_body_mutex;

extern deque<VelocityNED> velocity_topic;
extern mutex velocity_mutex;

extern deque<EulerAngle> attitude_topic;
extern mutex attitude_mutex;

extern vector<Reference> reference_topic;
extern mutex reference_mutex;

extern vector<InputVelocityBody> input_velocity_body_topic;
extern mutex input_velocity_body_mutex;

extern vector<InputAttitude> input_attitude_topic;
extern mutex input_attitude_mutex;

extern vector<Status> status_topic;
extern mutex status_mutex;

extern vector<string> string_topic;
extern mutex string_mutex;

extern deque<DetectionResult> target_topic;
extern mutex target_mutex;

void sendLoop( FileNode send_config );
bool sendSocketInit();

void sendMsg( uint8_t msg_type, uint16_t length, void* buffer );
bool compress( Mat image, double resize_k, int quality, vector<uchar>& img_buffer );

void sendHeartBeat( void );
void sendImg( bool gray, double img_msg_resize, int img_msg_quality );
void sendPosition( void );
void sendPositionBody(void);
void sendVelocity( void );
void sendAttitude( void );
void sendStatus( void );
void sendString( void );
void sendInputAttitude( void );
void sendReference(void);
void sendTarget(void);
#endif