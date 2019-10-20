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
#include "clock.hpp"
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

extern vector<Input> input_vec_topic;
extern mutex input_vec_mutex;

extern vector<PositionNED> position_vec_log_topic;
extern mutex position_vec_log_mutex;

extern vector<VelocityNED> velocity_vec_log_topic;
extern mutex velocity_vec_log_mutex;

extern vector<EulerAngle> euler_angle_vec_log_topic;
extern mutex euler_angle_vec_log_mutex;

extern vector<Input> input_vec_log_topic;
extern mutex input_vec_log_mutex;

extern GCCommand command_topic;
extern mutex command_mutex;

void controlLoop( FileNode control_config );
void setTelemetry( shared_ptr<Telemetry> telemetry );

void healthCheck( shared_ptr<Telemetry> telemetry );
void arm( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action );
void takeoff( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action, float altitude );
void land( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action );

void ctrlVelocityBody( std::shared_ptr<mavsdk::Offboard> offboard, Offboard::VelocityBodyYawspeed u );
void clearCommand();

void waitForArmed( shared_ptr<Telemetry> telemetry );
void waitForOffboard( shared_ptr<Offboard> offboard );
void pushInput( Input input );
void test( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action, shared_ptr<Offboard> offboard );

inline void offboard_log(const std::string& offb_mode, const std::string msg);
inline void offboard_error_exit(Offboard::Result result, const std::string& message);
bool offb_ctrl_attitude(std::shared_ptr<mavsdk::Offboard> offboard);
#endif
