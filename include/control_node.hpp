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

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

using namespace std;
using namespace mavsdk;
using namespace cv;
using namespace std::chrono;

//command
extern vector<MissionCommand> mission_command_topic;
extern mutex mission_command_mutex;


void controlLoop( FileNode control_config );

void altitudeTest( shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, float Kp_z, float Ki_z, float Kd_z );
float altitudeThrustControl( float altitude, shared_ptr<Telemetry> telemetry, float dt );

#endif
