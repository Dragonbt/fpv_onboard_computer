#include "control_node.hpp"
#include "const.hpp"
#include <vector>
using namespace std;

void testLoop( shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, FileNode altitude_pid, FileNode vision_pid, FileNode flow_pid )
{
	int16_t status = WAIT_COMMAND;
	int64_t command_timestamp, target_timestamp;
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
	FlowPosThrustControl flow_pos_thrust_control;
	altitude_thrust_control.reset(altitude_pid);
	vision_roll_thrust_control.reset(vision_pid, altitude_pid);
	flow_pos_thrust_control.reset(flow_pid, altitude_pid);

	Offboard::Attitude input_attitude;
	float roll_deg, pitch_deg, yaw_deg, thrust;
	float pos_sp_z;
	float open_loop_distance;
	int period_ms;
	high_resolution_clock::time_point open_loop_t0;
	int64_t last_peroid = timestampf();
	int fail_cnt = 0;
	Vector3f offset_body;
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
		if( mission_command_topic.latest(command_timestamp, command) )
		{
			mission_command_topic.clear();
			status = command.index;
			param = command.argv[0];
		}
		else{
			param = 0.0;
		}

		if( -position_ned.down_m > ALTITUDE_UPPER_BOUND )
		{
			if(status != FORCE_QUIT_COMMAND)
				status = SAFE_QUIT_COMMAND;
		}

		//ignore compute time, this loop update every 20ms
		control_status_topic.update(status);
		switch( status )
		{
			case WAIT_COMMAND:
				roll_deg = attitude.roll_deg;
				pitch_deg = attitude.pitch_deg;
				yaw_deg = attitude.yaw_deg;
				pos_sp_z = position_ned.down_m;
				//do nothing
				break;
			case SAFE_QUIT_COMMAND:
				//remotePrint("[WARNIGN]: LANDING!");
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
				//thrust = altitude_thrust_control.hold(position_ned, velocity_body, attitude, period_ms);
				altitude_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
				//cout << thrust << endl;
				offbCtrlAttitude(offboard, input_attitude);
				break;
			case FLOW_HOLD_COMMAND:
			    offset_body = {0.0f, 0.0f, 0.0f};
				flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
				status = FLOW_HOLD_MODE;
				remotePrint("Flow Hold!");
				break;
			case FLOW_HOLD_MODE:
				flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
				break;
			case VISION_CONTROL_COMMAND:
				if ( target_topic.latest(target_timestamp, target) && timestampf() - target_timestamp < 30){
					remotePrint("VISION CONTROL!");
					status = VISION_CONTROL_MODE;
					vision_roll_thrust_control.angleOffset(roll_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
					input_attitude = {roll_deg, -2.0f, yaw_deg, thrust};
					offbCtrlAttitude(offboard, input_attitude);
					fail_cnt = 0;
				}
				else{
					thrust = altitude_thrust_control.down(pos_sp_z, position_ned, velocity_body, attitude, period_ms);
					input_attitude = {0.0f, 0.0f, yaw_deg, thrust};
					offbCtrlAttitude(offboard, input_attitude);
				}
				break;
			case VISION_CONTROL_MODE:
				if( target_topic.latest(target_timestamp, target) && timestampf() - target_timestamp < 30 )
				{
					vision_roll_thrust_control.angleOffset(roll_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
					fail_cnt = 0;
					vision_roll_thrust_control.update_thr_ring_flag(target);
				}
				else{
					vision_roll_thrust_control.hold(roll_deg, thrust, position_ned, velocity_body, attitude, period_ms);
					fail_cnt ++;
				}
				if( fail_cnt > VISION_FAIL_TOLERENCE )//1s?
				{
					if(vision_roll_thrust_control.can_through_ring_flag){
						status = VISION_OPEN_LOOP_MODE;
						cout << "VISION OPEN LOOP!" << endl;
						break;
					}
					else{
						remotePrint("TIMEOUT!");
						//vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, pos_ned, vel_body, attitude, period_ms);
						status = BRAKING;
						//status = SAFE_QUIT_COMMAND;
						//status = SAFE_QUIT_COMMAND;
						break;
					}
					
				}
				input_attitude = {roll_deg, -2.0f, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
				break;
			case VISION_OPEN_LOOP_MODE:
				fail_cnt++;
				if(fail_cnt < 1.5*VISION_FAIL_TOLERENCE){
					vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
					input_attitude = {roll_deg, -2.0f, yaw_deg, thrust};
					offbCtrlAttitude(offboard, input_attitude);
				}
				else {
					vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
					input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
					offbCtrlAttitude(offboard, input_attitude);
					if(fail_cnt > 2.5*VISION_FAIL_TOLERENCE) {
						status = SEARCH_RING;
						cout << "SEARCH RING!" << endl;
						break;
					}

				}
				break;
			case BRAKING:
				cout << "BRAKING" << endl;
				vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
				break;
			case SEARCH_RING:
				vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
				break;
		}
		this_thread::sleep_for(milliseconds(20));
		period_ms = timestampf() - last_peroid;
		last_peroid = timestampf();
	}	
    return;
}