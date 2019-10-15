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
int img_msg_quality = 60;
double img_msg_resize = 0.2;

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

/*program entrance*/
int main()
{
    thread camera_node( cameraLoop, 0, 1280, 720 );
    //thread camera_node_test( cameraLoopTest );
    thread send_node( sendLoop, "127.0.0.1", 8080 );
    thread recv_node( recvLoop, "127.0.0.1", 8080 );
    thread log_node( logLoop, "./" );
    camera_node.join();
    //camera_node_test.join();
    send_node.join();
    recv_node.join();
    log_node.join();
    return 0;
}