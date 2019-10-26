#ifndef _TELEM_
#define _TELEM_

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <mutex>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "utils.hpp"
#include "struct.hpp"

#define MAX_VEC_SIZE 40

using namespace std;
using namespace mavsdk;
using namespace std::chrono;

extern high_resolution_clock::time_point init_timepoint;

//telem to send
extern deque<PositionNED> position_topic;
extern mutex position_mutex;

extern deque<VelocityNED> velocity_topic;
extern mutex velocity_mutex;

extern deque<EulerAngle> attitude_topic;
extern mutex attitude_mutex;

extern vector<Status> status_topic;
extern mutex status_mutex;

void setTelemetry( shared_ptr<Telemetry> telemetry );

#endif