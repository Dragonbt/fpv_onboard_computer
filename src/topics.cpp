#include "struct.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <opencv2/core/core.hpp>
#include <chrono>
#include "socket_nodes.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono;

/*declaration of shared constant*/
high_resolution_clock::time_point init_timepoint = high_resolution_clock::now();

/*declaration of shared topics( or variables ) between nodes,
CAUTIOUS: every topics must own a mutex*/
/*camera_node*/
Mat image_topic;
mutex image_mutex;

int camera_status_topic = 0;
mutex camera_status_mutex;

int camera_exception_topic = 0;
mutex camera_exception_mutex;

/*log_node*/
LogStatus log_status_topic;
mutex log_status_mutex;

vector<PositionNED> position_vec_topic;
mutex position_vec_mutex;

vector<VelocityNED> velocity_vec_topic;
mutex velocity_vec_mutex;

vector<EulerAngle> euler_angle_vec_topic;
mutex euler_angle_vec_mutex;

vector<Input> u_vec_topic;
mutex u_vec_mutex;

/*sockets_node*/
int socket_exception_topic = 0;
mutex socket_exception_mutex;

int fd = -1;
mutex fd_mutex;
struct sockaddr_in send_to_addr;
struct sockaddr_in recv_from_addr;

/*control_node*/
PositionNED position_topic;
mutex position_mutex;

VelocityNED velocity_topic;
mutex velocity_mutex;

EulerAngle euler_angle_topic;
mutex euler_angle_mutex;

GCCommand command_topic;
mutex command_mutex;