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

#include "clock.hpp"
#include <thread>
#include <mutex>

#include "log_node.hpp"

#define PEER_SHUTDOWN -1
#define MAX_MSG_CONTENT_LENGTH 0xFFFF
#define HEAD 0xAAAA
#define TAIL 0xDDDD

using namespace std;
using namespace cv;

extern high_resolution_clock::time_point init_timepoint;
extern int img_msg_quality;
extern double img_msg_resize;

extern Mat image_topic;
extern mutex image_mutex;

extern int camera_status_topic;
extern mutex camera_status_mutex;

extern int socket_exception_topic;
extern mutex socket_exception_mutex;

extern LogStatus log_status_topic;
extern mutex log_status_mutex;

void sendLoop( const char* host, int port );
void recvLoop( const char* host, int port );

bool sendMsg( int fd, int64_t timepoint_ms, uint8_t msg_type, uint16_t length, void* buffer );
bool compress( Mat image, vector<uchar>& img_buffer );

bool udpServerInit( int& sock_fd, const char* host, const int port );
bool udpClientInit( int& sock_fd, const char* host, const int port );

bool tcpServerInit( int& server_fd, int& client_fd, const char* host, const int port);
bool tcpClientInit( int& client_fd, const char* host, const int port );

bool sendAll( int socket, void *buffer, size_t length );
bool recvAll( int socket, void *buffer, size_t length );

#endif