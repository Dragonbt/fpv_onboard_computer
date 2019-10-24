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

#include "telem.hpp"

using namespace std;
using namespace mavsdk;
using namespace cv;
using namespace std::chrono;

extern high_resolution_clock::time_point init_timepoint;

//command
extern MissionCommand mission_command_topic;
extern mutex mission_command_mutex;


void controlLoop( FileNode control_config );

void healthCheck( shared_ptr<Telemetry> telemetry );
//void arm( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action );
//void takeoff( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action, float altitude );
//void land( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action );

void waitForArmed( shared_ptr<Telemetry> telemetry );
void quitOffboard( shared_ptr<Offboard> offboard );

void pushInputVelocityBody( Offboard::VelocityBodyYawspeed velocity );
void pushInputAttitude( Offboard::Attitude attitude );

void altitudeTest( shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard );
void altitude(shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, double SampleTime);

void offbCtrlAttitude(shared_ptr<Offboard> offboard, Offboard::Attitude attitude);
void offbCtrlVelocityBody( std::shared_ptr<mavsdk::Offboard> offboard, Offboard::VelocityBodyYawspeed velocity );
void offbCtrlPositionNED( std::shared_ptr<mavsdk::Offboard> offboard, Offboard::PositionNEDYaw position );
#endif
