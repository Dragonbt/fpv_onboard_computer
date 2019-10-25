#include "struct.hpp"
#include <opencv2/core/core.hpp>
#include <chrono>
#include <unistd.h>
#include <linux/in.h>
#include <mutex>
#include <iostream>
#include <fstream>

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

vector<InputVelocityBody> input_velocity_body_vec_topic;
mutex input_velocity_body_vec_mutex;

vector<InputAttitude> input_attitude_vec_topic;
mutex input_attitude_vec_mutex;

vector<string> string_vec_topic;
mutex string_vec_mutex;

vector<Reference> reference_topic;
mutex reference_mutex;

/*control_node*/
vector<Status> status_topic;
mutex status_mutex;

MissionCommand mission_command_topic;
mutex mission_command_mutex;

ofstream log_file_topic;
mutex log_file_mutex;