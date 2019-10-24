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

#include "utils.hpp"
#include "struct.hpp"

#include "flight_control_sync.hpp"
#include "telemetry_async.hpp"

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

using namespace std;
using namespace mavsdk;
using namespace cv;
using namespace std::chrono;

//command
extern MissionCommand mission_command_topic;
extern mutex mission_command_mutex;


void controlLoop( FileNode control_config );

void altitudeTest( shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, double P, double D );
void altitude(shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, double SampleTime);

#endif
