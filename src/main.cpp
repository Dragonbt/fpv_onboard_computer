#include <iostream>
#include <thread>
#include <mutex>

#include "camera_node.hpp"
#include "socket_nodes.hpp"
#include "log_node.hpp"


using namespace std;
using namespace cv;

/*declaration of shared constant*/
high_resolution_clock::time_point init_timepoint = high_resolution_clock::now();

/*declaration of shared topics( or variables ) between nodes,
CAUTIOUS: every topics must own a mutex*/
Mat image_topic;
mutex image_mutex;

int camera_status_topic = 0;
mutex camera_status_mutex;

int camera_exception_topic = 0;
mutex camera_exception_mutex;

LogStatus log_status_topic;
mutex log_status_mutex;

int socket_exception_topic = 0;
mutex socket_exception_mutex;

string getAbsolutePath( void );

/*program entrance*/
int main( int argc,char *argv[] )
{
    FileStorage config( "../config/config.yaml", FileStorage::READ );
    FileNode camera_config = config["CAMERA_NODE"];
    FileNode send_config = config["SOCKET_SEND_NODE"];
    FileNode recv_config = config["SOCKET_RECV_NODE"];
    FileNode log_config = config["LOG_NODE"];

    thread camera_node( cameraLoop, camera_config );
    //thread camera_node_test( cameraLoopTest );
    thread socket_send_node( sendLoop, send_config );
    thread socket_recv_node( recvLoop, recv_config );
    thread log_node( logLoop, log_config );
    camera_node.join();
    //camera_node_test.join();
    socket_send_node.join();
    socket_recv_node.join();
    log_node.join();

    return 0;
}