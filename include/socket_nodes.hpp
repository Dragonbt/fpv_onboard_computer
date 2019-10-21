#ifndef _SOCKET_NODES_
#define _SOCKET_NODES_

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

#include "clock.hpp"
#include "struct.hpp"

using namespace std;
using namespace cv;

extern int fd;
extern mutex fd_mutex;
extern struct sockaddr_in send_to_addr;
extern struct sockaddr_in recv_from_addr;

extern high_resolution_clock::time_point init_timepoint;
extern Mat image_topic;
extern mutex image_mutex;

extern int camera_status_topic;
extern mutex camera_status_mutex;

extern int socket_exception_topic;
extern mutex socket_exception_mutex;

extern vector<PositionNED> position_vec_topic;
extern mutex position_vec_mutex;

extern vector<VelocityNED> velocity_vec_topic;
extern mutex velocity_vec_mutex;

extern vector<EulerAngle> euler_angle_vec_topic;
extern mutex euler_angle_vec_mutex;

extern vector<Input> input_vec_topic;
extern mutex input_vec_mutex;

extern GCCommand command_topic;
extern mutex command_mutex;

void sendLoop( FileNode send_config );
void recvLoop( FileNode recv_config );

bool socketInit();

void sendMsg( uint8_t msg_type, uint16_t length, void* buffer );
void recvMsg( char* msg, int msg_length, sockaddr_in address );
bool compress( Mat image, double resize_k, int quality, vector<uchar>& img_buffer );

void sendHeartBeat( void );
void sendImg( double img_msg_resize, int img_msg_quality );
void sendPosition( void );
void sendVelocity( void );
void sendAttitude( void );
void sendStatus( void );

#endif