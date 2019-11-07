#include "control_node.hpp"
#include "const.hpp"
#include <vector>
using namespace std;

void testLoop( shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, FileNode altitude_pid, FileNode vision_pid, FileNode flow_pid )
{
	int16_t status = WAIT_COMMAND;
	int64_t timestamp;
	double param;
	MissionCommand command;
	Telemetry::PositionVelocityNED position_velocity_ned;
	Telemetry::EulerAngle euler_angle;
	PositionNED position_ned;
	VelocityNED velocity_ned;
	VelocityBody velocity_body;
	EulerAngle attitude;
	DetectionResult target;

	AltitudeThrustControl altitude_thrust_control;
	VisionRollThrustControl vision_roll_thrust_control;
	altitude_thrust_control.reset(altitude_pid);
	vision_roll_thrust_control.reset(vision_pid, altitude_pid);

	Offboard::Attitude input_attitude;
	float roll_deg, pitch_deg, yaw_deg, thrust;

	int period_ms;
	int64_t last_peroid = timestampf();
	int64_t last_seen = timestampf();
    while(true)
    {
		//only read telemetry data once every iteration
		position_velocity_ned = telemetry->position_velocity_ned();
		euler_angle = telemetry->attitude_euler_angle();
		memcpy(&position_ned, &position_velocity_ned.position, sizeof position_ned);
		memcpy(&velocity_ned, &position_velocity_ned.velocity, sizeof velocity_ned);
		memcpy(&attitude, &euler_angle, sizeof attitude);
		velocity_body = velocityNED2Body(velocity_ned, attitude);

		//receive ground control command
		if( mission_command_topic.latest(timestamp, command) )
		{
			mission_command_topic.clear();
			status = command.index;
			param = command.argv[0];
			control_status_topic.update(status);
		}
		else{
			param = 0.0;
		}

		if( -position_ned.down_m > ALTITUDE_UPPER_BOUND )
		{
			status = SAFE_QUIT_COMMAND;
		}

		//ignore compute time, this loop update every 20ms
		switch( status )
		{
			case WAIT_COMMAND:
				roll_deg = attitude.roll_deg;
				pitch_deg = attitude.pitch_deg;
				yaw_deg = attitude.yaw_deg;
				//do nothing
				break;
			case SAFE_QUIT_COMMAND:
				remotePrint("[WARNIGN]: LANDING!");
				thrust = altitude_thrust_control.landing();
				input_attitude = {0.0f, 0.0f, attitude.yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
				break;
			case FORCE_QUIT_COMMAND:
				remotePrint("[WARNIGN]: QUIT OFFBOARD!");
				quitOffboard( offboard );
				return;
			case ALTITUDE_STEP_COMMAND:
				remotePrint("[LOGGING]: ALTITUDE STEP COMMAND!");
				status = ALTITUDE_STAY_MODE;
				cout << param << endl;
				thrust = altitude_thrust_control.downOffset(-param, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {0.0f, 0.0f, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
				break;
			case ALTITUDE_STAY_MODE:
				thrust = altitude_thrust_control.hold(position_ned, velocity_body, attitude, period_ms);
				input_attitude = {0.0f, 0.0f, yaw_deg, thrust};
				//cout << thrust << endl;
				offbCtrlAttitude(offboard, input_attitude);
				break;
			case VISION_CONTROL_COMMAND:
				target_topic.latest(timestamp, target);
				if( target.confidence > 0 && timestampf() - timestamp < 30 )
				{
					last_seen = timestampf();
					vision_roll_thrust_control.angleOffset(roll_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
				}
				else{
					if( timestampf() - last_seen > 1000 )
					{
						remotePrint("[WARNING]: TIME OUT!");
						status = SAFE_QUIT_COMMAND;
						break;
					}
					vision_roll_thrust_control.hold(roll_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				}
				input_attitude = {roll_deg, 0.0f, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
				break;
		}
		this_thread::sleep_for(milliseconds(20));
		timestamp = timestampf();
		period_ms = timestamp - last_peroid;
		last_peroid = timestamp;	
	}
    return;
}