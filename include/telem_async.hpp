#ifndef _TELEM_
#define _TELEM_

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <math.h>
#include <mutex>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "utils.hpp"
#include "struct.hpp"

using namespace std;
using namespace mavsdk;
using namespace std::chrono;

extern Topic<PositionNED> position_ned_topic;

extern Topic<PositionBody> position_body_topic;

extern Topic<VelocityNED> velocity_ned_topic;

extern Topic<VelocityBody> velocity_body_topic;

extern Topic<EulerAngle> attitude_topic;

extern Topic<VehicleStatus> vehicle_status_topic;

void setTelemetry( shared_ptr<Telemetry> telemetry );

#endif