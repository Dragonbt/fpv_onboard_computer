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

/*declaration of shared topics( or variables ) between nodes,
CAUTIOUS: every topics must own a mutex*/
/*camera_node*/
Mat image_topic;
mutex image_mutex;

int camera_status_topic = 1;
mutex camera_status_mutex;

deque<DetectionResult> target_topic;
mutex target_mutex;

int camera_exception_topic = 0;
mutex camera_exception_mutex;

/*sockets_node*/
int fd = -1;
mutex fd_mutex;

deque<PositionNED> position_topic;
mutex position_mutex;

deque<PositionBody> position_body_topic;
mutex position_body_mutex;

deque<VelocityNED> velocity_topic;
mutex velocity_mutex;

deque<EulerAngle> attitude_topic;
mutex attitude_mutex;

vector<InputVelocityBody> input_velocity_body_topic;
mutex input_velocity_body_mutex;

vector<InputAttitude> input_attitude_topic;
mutex input_attitude_mutex;

vector<Reference> reference_topic;
mutex reference_mutex;

vector<string> string_topic;
mutex string_mutex;

/*control_node*/
vector<Status> status_topic;
mutex status_mutex;

vector<MissionCommand> mission_command_topic;
mutex mission_command_mutex;

ofstream log_file_topic;
mutex log_file_mutex;