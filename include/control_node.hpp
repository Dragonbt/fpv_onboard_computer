#ifndef _CONTROL_NODE_
#define _CONTROL_NODE_

#include <iostream>
#include <stdlib.h>
#include <string>
#include <thread>
#include <mutex>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <opencv2/core/core.hpp>

#include "utils.hpp"
#include "struct.hpp"

#include "control_sync.hpp"
#include "telem_async.hpp"
#include "topic.hpp"

using namespace std;
using namespace mavsdk;
using namespace cv;
using namespace std::chrono;

//command
extern Topic<MissionCommand> mission_command_topic;

extern Topic<int16_t> control_status_topic;

extern Topic<DetectionResult> target_topic;

float rad2deg(float rad);
void controlLoop( FileNode control_config );

void altitudeTest( shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, float Kp_z, float Ki_z, float Kd_z,float Kp_x, float Ki_x, float Kd_x,float Kp_y, float Ki_y, float Kd_y );
vector<float> positionThrustControl(vector<float> _pos_sp, shared_ptr<Telemetry> telemetry, float dt);
float altitudeThrustControl( float altitude, shared_ptr<Telemetry> telemetry, float dt );

#endif
