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

using namespace std;
using namespace mavsdk;
using namespace std::chrono;

extern high_resolution_clock::time_point init_timepoint;

//telem to send
extern vector<PositionNED> position_vec_topic;
extern mutex position_vec_mutex;

extern vector<VelocityNED> velocity_vec_topic;
extern mutex velocity_vec_mutex;

extern vector<EulerAngle> euler_angle_vec_topic;
extern mutex euler_angle_vec_mutex;

extern vector<InputVelocityBody> input_velocity_body_vec_topic;
extern mutex input_velocity_body_vec_mutex;

extern vector<InputAttitude> input_attitude_vec_topic;
extern mutex input_attitude_vec_mutex;

static Status status;
extern vector<Status> status_topic;
extern mutex status_mutex;

//telem to log
extern vector<PositionNED> position_vec_log_topic;
extern mutex position_vec_log_mutex;

extern vector<VelocityNED> velocity_vec_log_topic;
extern mutex velocity_vec_log_mutex;

extern vector<EulerAngle> euler_angle_vec_log_topic;
extern mutex euler_angle_vec_log_mutex;

extern vector<InputVelocityBody> input_velocity_body_vec_log_topic;
extern mutex input_velocity_body_vec_log_mutex;

extern vector<InputAttitude> input_attitude_vec_log_topic;
extern mutex input_attitude_vec_log_mutex;

void setTelemetry( shared_ptr<Telemetry> telemetry );

#endif