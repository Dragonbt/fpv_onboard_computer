#include <opencv2/core/core.hpp>
#include <chrono>
#include <unistd.h>
#include <linux/in.h>
#include <mutex>
#include <iostream>
#include <fstream>

#include "struct.hpp"
#include "topic.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono;

/*declaration of shared constant*/
high_resolution_clock::time_point init_timepoint = high_resolution_clock::now();

/*declaration of shared topics( or variables ) between nodes,
CAUTIOUS: every topics must own a mutex*/
/*sockets_node*/
int fd = -1;
mutex fd_mutex;

Topic<Mat> image_topic(init_timepoint, 1);

Topic<int> camera_status_topic(init_timepoint, 1);

Topic<DetectionResult> target_topic(init_timepoint, 10);

Topic<string> string_topic(init_timepoint, 5);

Topic<PositionNED> position_ned_topic(init_timepoint, 40);

Topic<VelocityNED> velocity_ned_topic(init_timepoint, 40);

Topic<VelocityBody> velocity_body_topic(init_timepoint, 40);

Topic<EulerAngle> attitude_topic(init_timepoint, 40);

Topic<VehicleStatus> vehicle_status_topic(init_timepoint, 40);

//Topic<InputVelocityBody> input_velocity_body_topic(init_timepoint, 40);
Topic<InputAttitude> input_attitude_topic(init_timepoint, 40);

Topic<float> down_reference_topic(init_timepoint, 40);

Topic<Vector2f> ne_reference_topic(init_timepoint, 40);

Topic<MissionCommand> mission_command_topic(init_timepoint, 1);

Topic<int16_t> control_status_topic(init_timepoint, 10);

Topic<Vector2f> pos_err_xy_topic(init_timepoint, 40);