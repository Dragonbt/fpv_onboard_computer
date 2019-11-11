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
extern mutex mission_command_mtx;
extern Topic<MissionCommand> mission_command_topic;

extern mutex control_status_mtx;
extern Topic<int16_t> control_status_topic;

extern mutex target_mtx;
extern Topic<DetectionResult> target_topic;

float rad2deg(float rad);
void controlLoop( FileNode control_config, FileNode altitude_pid, FileNode vision_pid, FileNode flow_pid );

void testLoop( shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, FileNode altitude_pid, FileNode vision_pid, FileNode flow_pid );

#endif
