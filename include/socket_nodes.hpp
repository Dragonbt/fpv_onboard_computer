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
#include "log_node.hpp"

#define PEER_SHUTDOWN -1
#define MAX_MSG_LENGTH 0x22FF
#define HEAD 0xAAAA
#define TAIL 0xDDDD

using namespace std;
using namespace cv;

static int fd = -1;
static mutex fd_mutex;
static struct sockaddr_in send_to_addr;
static struct sockaddr_in recv_from_addr;

extern high_resolution_clock::time_point init_timepoint;
extern Mat image_topic;
extern mutex image_mutex;

extern int camera_status_topic;
extern mutex camera_status_mutex;

extern int socket_exception_topic;
extern mutex socket_exception_mutex;

extern LogStatus log_status_topic;
extern mutex log_status_mutex;

void sendLoop( FileNode send_config );
void recvLoop( FileNode recv_config );

bool socketInit();

void sendMsg( uint8_t msg_type, uint16_t length, void* buffer );
void recvMsg( char* msg, int msg_length, sockaddr_in address );
bool compress( Mat image, double resize_k, int quality, vector<uchar>& img_buffer );

#endif