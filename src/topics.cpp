#include "struct.hpp"
#include <opencv2/core/core.hpp>
#include <chrono>
#include <unistd.h>
#include <linux/in.h>
#include <mutex>

using namespace std;
using namespace cv;
using namespace std::chrono;

/*declaration of shared constant*/
high_resolution_clock::time_point init_timepoint = high_resolution_clock::now();
struct sockaddr_in send_to_addr;
struct sockaddr_in recv_from_addr;

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
vector<PositionNED> position_vec_log_topic;
mutex position_vec_log_mutex;

vector<VelocityNED> velocity_vec_log_topic;
mutex velocity_vec_log_mutex;

vector<EulerAngle> euler_angle_vec_log_topic;
mutex euler_angle_vec_log_mutex;

vector<Input> input_vec_log_topic;
mutex input_vec_log_mutex;

/*sockets_node*/
int socket_exception_topic = 0;
mutex socket_exception_mutex;

int fd = -1;
mutex fd_mutex;

vector<PositionNED> position_vec_topic;
mutex position_vec_mutex;

vector<VelocityNED> velocity_vec_topic;
mutex velocity_vec_mutex;

vector<EulerAngle> euler_angle_vec_topic;
mutex euler_angle_vec_mutex;

vector<Input> input_vec_topic;
mutex input_vec_mutex;

/*control_node*/
GCCommand command_topic;
mutex command_mutex;