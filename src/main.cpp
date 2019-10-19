#include <iostream>
#include <thread>
#include <mutex>

#include "camera_node.hpp"
#include "socket_nodes.hpp"
#include "log_node.hpp"
#include "control_node.hpp"

using namespace std;
using namespace cv;

/*program entrance*/
int main( int argc, char* argv[] )
{
    FileStorage config;
    if( argc == 2 && ! strcmp(argv[1], "desktop"))
    {
        config.open( "../config/config_desktop.yaml", FileStorage::READ );
    }
    else{
        config.open( "../config/config.yaml", FileStorage::READ );
    }
    FileNode camera_config = config["CAMERA_NODE"];
    FileNode send_config = config["SOCKET_SEND_NODE"];
    FileNode recv_config = config["SOCKET_RECV_NODE"];
    FileNode log_config = config["LOG_NODE"];
    FileNode control_config = config["CONTROL_NODE"];

    thread camera_node( cameraLoop, camera_config );
    //thread camera_node_test( cameraLoopTest );
    thread socket_send_node( sendLoop, send_config );
    thread socket_recv_node( recvLoop, recv_config );
    thread log_node( logLoop, log_config );
    thread control_node( controlLoop, control_config );
    camera_node.join();
    //camera_node_test.join();
    socket_send_node.join();
    socket_recv_node.join();
    control_node.join();
    log_node.join();

    return 0;
}