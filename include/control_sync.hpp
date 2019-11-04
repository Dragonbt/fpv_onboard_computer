#include <thread>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "utils.hpp"
#include "struct.hpp"

using namespace std;
using namespace mavsdk;
using namespace chrono;

extern Topic<InputAttitude> input_attitude_topic;

extern Topic<double> down_reference_topic;

void healthCheck( shared_ptr<Telemetry> telemetry );
void waitForArmed( shared_ptr<Telemetry> telemetry );
void quitOffboard( shared_ptr<Offboard> offboard );

void offbCtrlAttitude(shared_ptr<Offboard> offboard, Offboard::Attitude attitude);
void offbCtrlPositionNED( std::shared_ptr<mavsdk::Offboard> offboard, Offboard::PositionNEDYaw position );