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

#include "utils.hpp"
#include "struct.hpp"

#define MAX_MSG_LENGTH 0x22FF
#define HEAD 0xAAAA
#define TAIL 0xDDDD

//send msg type
#define HEART_BEAT_MSG 0
#define IMG_MSG 1
#define POSITION_MSG 2
#define VELOCITY_MSG 3
#define ATTITUDE_MSG 4
#define INPUT_MSG 5
#define STATUS_MSG 6
#define LOG_MSG 7
#define REFERENCE_MSG 8

//recv command type
#define MISSION_COMMAND_MSG 15
#define VIDEO_COMMAND_MSG 5
#define LOG_COMMAND_MSG 6

using namespace std;
using namespace cv;
using namespace chrono;

extern int fd;
extern mutex fd_mutex;
extern struct sockaddr_in send_to_addr;
extern struct sockaddr_in recv_from_addr;

extern high_resolution_clock::time_point init_timepoint;
extern Mat image_topic;
extern mutex image_mutex;

extern int socket_exception_topic;
extern mutex socket_exception_mutex;

extern vector<PositionNED> position_vec_topic;
extern mutex position_vec_mutex;

extern vector<VelocityNED> velocity_vec_topic;
extern mutex velocity_vec_mutex;

extern vector<EulerAngle> euler_angle_vec_topic;
extern mutex euler_angle_vec_mutex;

extern vector<InputVelocityBody> input_velocity_body_vec_topic;
extern mutex input_velocity_body_vec_mutex;

extern vector<InputAttitude> input_attitude_vec_topic;
extern mutex input_attitude_vec_mutex;

extern MissionCommand mission_command_topic;
extern mutex mission_command_mutex;

extern vector<Status> status_topic;
extern mutex status_mutex;

extern vector<Reference> reference_topic;
extern mutex reference_mutex;

void sendLoop( FileNode send_config );
void recvLoop( FileNode recv_config );

bool socketInit();

void sendMsg( uint8_t msg_type, uint16_t length, void* buffer );
void recvMsg( char* msg, int msg_length, sockaddr_in address );
bool compress( Mat image, double resize_k, int quality, vector<uchar>& img_buffer );

void sendHeartBeat( void );
void sendImg( bool gray, double img_msg_resize, int img_msg_quality );
void sendPosition( void );
void sendVelocity( void );
void sendAttitude( void );
void sendStatus( void );
void sendString( void );
void sendInputAttitude( void );
void sendReference(void);
#endif