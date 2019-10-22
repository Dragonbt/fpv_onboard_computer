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

using namespace std;
using namespace mavsdk;
using namespace cv;
using namespace std::chrono;

extern high_resolution_clock::time_point init_timepoint;

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

extern GCCommand command_topic;
extern mutex command_mutex;

static Status status;
extern vector<Status> status_topic;
extern mutex status_mutex;

void controlLoop( FileNode control_config );
void setTelemetry( shared_ptr<Telemetry> telemetry );

void healthCheck( shared_ptr<Telemetry> telemetry );
void arm( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action );
void takeoff( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action, float altitude );
void land( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action );

bool readControlCommand( GCCommand &command );

void waitForArmed( shared_ptr<Telemetry> telemetry );
void quitOffboard( shared_ptr<Offboard> offboard );
void pushInputVelocityBody( Offboard::VelocityBodyYawspeed velocity );
void pushInputAttitude( Offboard::Attitude attitude );
void test( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action, shared_ptr<Offboard> offboard );

void offbCtrlAttitude(shared_ptr<Offboard> offboard, Offboard::Attitude attitude);
void offbCtrlVelocityBody( std::shared_ptr<mavsdk::Offboard> offboard, Offboard::VelocityBodyYawspeed velocity );
void offbCtrlPositionNED( std::shared_ptr<mavsdk::Offboard> offboard, Offboard::PositionNEDYaw position );
#endif
