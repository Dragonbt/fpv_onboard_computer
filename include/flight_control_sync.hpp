
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "utils.hpp"
#include "struct.hpp"

using namespace std;
using namespace mavsdk;
using namespace chrono;

extern high_resolution_clock::time_point init_timepoint;

extern vector<InputVelocityBody> input_velocity_body_vec_topic;
extern mutex input_velocity_body_vec_mutex;

extern vector<InputAttitude> input_attitude_vec_topic;
extern mutex input_attitude_vec_mutex;

void waitForArmed( shared_ptr<Telemetry> telemetry );
void quitOffboard( shared_ptr<Offboard> offboard );

void offbCtrlAttitude(shared_ptr<Offboard> offboard, Offboard::Attitude attitude);
void offbCtrlVelocityBody( std::shared_ptr<mavsdk::Offboard> offboard, Offboard::VelocityBodyYawspeed velocity );
void offbCtrlPositionNED( std::shared_ptr<mavsdk::Offboard> offboard, Offboard::PositionNEDYaw position );

void healthCheck( shared_ptr<Telemetry> telemetry );

void pushInputVelocityBody( Offboard::VelocityBodyYawspeed velocity );
void pushInputAttitude( Offboard::Attitude attitude );
