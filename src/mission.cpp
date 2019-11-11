#include "control_node.hpp"
#include "const.hpp"
#include <vector>
using namespace std;

#if (TEST_LEVEL == LEVEL_FINAL0) || (TEST_LEVEL == LEVEL_FINAL1) || (TEST_LEVEL == LEVEL_FINAL2) || (TEST_LEVEL == LEVEL_FINAL3)
void missionLoop(shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, FileNode altitude_pid, FileNode vision_pid, FileNode flow_pid)
{
	int16_t missions_status = WAIT_COMMAND;
	int count = 0;

	float altitude_offset = 0;
	float altitude_set = 0;
	bool flag_climb_init = false, flag_search_init = false, flag_target1_found = false, flag_adjust_init = false, flag_land_init = false;

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
	while (true)
	{
		//only read telemetry data once every iteration
		position_velocity_ned = telemetry->position_velocity_ned();
		euler_angle = telemetry->attitude_euler_angle();
		memcpy(&position_ned, &position_velocity_ned.position, sizeof position_ned);
		memcpy(&velocity_ned, &position_velocity_ned.velocity, sizeof velocity_ned);
		memcpy(&attitude, &euler_angle, sizeof attitude);
		velocity_body = velocityNED2Body(velocity_ned, attitude);

		//receive ground control command
		if (latest<MissionCommand>(mission_command_topic, command_timestamp, command, mission_command_mtx))
		{
			clear<MissionCommand>(mission_command_topic, mission_command_mtx);
			missions_status = command.index;
			param = command.argv[0];
		}
		else
		{
			param = 0.0;
		}

		if (-position_ned.down_m > ALTITUDE_UPPER_BOUND)
		{
			if (missions_status != FORCE_QUIT_COMMAND)
				missions_status = SAFE_QUIT_COMMAND;
		}

		//ignore compute time, this loop update every 20ms
		update<int16_t>(control_status_topic, missions_status, control_status_mtx);

		//cout << missions_status << endl;
		switch (missions_status)
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
			quitOffboard(offboard);
			return;
		case INIT_MISSION:
			flag_climb_init = false;
			flag_target1_found = false;
			flag_adjust_init = false;
			flag_search_init = false;
			flag_land_init = false;
			altitude_offset = -position_ned.down_m;
			missions_status = TAKEOFF_MISSION;
			//sleep, waiting for system init
			break;
		case TAKEOFF_MISSION:
			cout << "TAKEOFF_MISSION" << endl;
			if (altitude_set < 1.1f)
				altitude_set += 0.5 / CONTROL_FREQUENCY;
			altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			//cout << thrust << endl;
			offbCtrlAttitude(offboard, input_attitude);
			if (-position_ned.down_m > 1.0f)
			{
				altitude_set = 1.0;
				missions_status = SETPOINT_CLIMB_MISSION;
			}
			break;
		case SETPOINT_CLIMB_MISSION:
			cout << "SETPOINT_CLIMB_MISSION" << endl;
			if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
			{
				cout << "FLOw CAN NOT THRUST" << endl;
				altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
				//cout << thrust << endl;
				offbCtrlAttitude(offboard, input_attitude);
				flag_climb_init = false;
			}
			else
			{
				if (!flag_climb_init)
				{
					altitude_set = -position_ned.down_m;
					offset_body = {0.0f, 0.0f, 0.0f};
					flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
					input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
					offbCtrlAttitude(offboard, input_attitude);
					remotePrint("Flow init!");
					flag_climb_init = true;
					break;
				}

				flow_pos_thrust_control.climb(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
			}
#if (TEST_LEVEL == LEVEL_FINAL1) || (TEST_LEVEL == LEVEL_FINAL2) || (TEST_LEVEL == LEVEL_FINAL3)
			if (flag_target1_found)
			{
				missions_status = AJUSTPOSITION_MISSION;
				break;
			}
			if (position_ned.down_m > 4.5)
			{
				missions_status = SEARCH_TARGET_MISSION;
				break;
			}
			if (altitude_set < 5)
				altitude_set += 0.75 / CONTROL_FREQUENCY;
#else
			if (-position_ned.down_m > 1.45 && altitude_set > 1.45)
			{
				altitude_set = 1.45;
				missions_status = AJUSTPOSITION_MISSION;
				break;
			}
			if (altitude_set < 1.5)
				altitude_set += 0.8 / CONTROL_FREQUENCY;
#endif
			break;
		case AJUSTPOSITION_MISSION:
			cout << "AJUSTPOSITION_MISSION" << endl;
#if (TEST_LEVEL == LEVEL_FINAL0)
			flow_pos_thrust_control.hold(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
			input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
			offbCtrlAttitude(offboard, input_attitude);
			count++;
			if (count >= 5 * CONTROL_FREQUENCY)
			{
				missions_status = LAND_MISSION;
			}
#else
			flag_target_lost

#endif
			break;
		case APROACH_MISSION:
			//flag_target_lost
			break;
		case THROUGH_MISSION:
			break;
		case STOP_MISSION:
			break;
		case SEARCH_TARGET_MISSION:
			break;
		case LAND_MISSION:
			//cout << "LAND_MISSION" << endl;
			if (-position_ned.down_m > 1.0)
			{
				if ((position_ned.east_m < 1e-4) && (position_ned.north_m < 1e-4))
				{
					altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
					input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
					//cout << thrust << endl;
					offbCtrlAttitude(offboard, input_attitude);
					flag_land_init = false;
				}
				else
				{
					if (!flag_land_init)
					{
						altitude_set = -position_ned.down_m;
						offset_body = {0.0f, 0.0f, 0.0f};
						flow_pos_thrust_control.positionBodyOffset(roll_deg, pitch_deg, thrust, offset_body, position_ned, velocity_body, attitude, period_ms);
						input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
						offbCtrlAttitude(offboard, input_attitude);
						remotePrint("land init!");
						flag_land_init = true;
						break;
					}
					flow_pos_thrust_control.climb(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
					input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
					offbCtrlAttitude(offboard, input_attitude);
				}
				altitude_set -= 0.8 / CONTROL_FREQUENCY;
			}
			else
			{
				altitude_set -= 0.3 / CONTROL_FREQUENCY;
				altitude_thrust_control.takeoff(-altitude_set, roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
				//cout << thrust << endl;
				offbCtrlAttitude(offboard, input_attitude);
			}
			if (altitude_set <= 0 && -position_ned.down_m < 0.2)
			{
				missions_status = FINISH;
			}
			break;
		case FINISH:
			input_attitude = {roll_deg, pitch_deg, yaw_deg, 0.0f};
			//cout << thrust << endl;
			offbCtrlAttitude(offboard, input_attitude);
			return;
		}
		this_thread::sleep_for(milliseconds(20));
		period_ms = timestampf() - last_peroid;
		last_peroid = timestampf();
	}
	return;
}
#else
void testLoop(shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, FileNode altitude_pid, FileNode vision_pid, FileNode flow_pid)
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
	float roll_deg, pitch_deg, yaw_deg, thrust, thrust_n;
	float pos_sp_z;
	//float open_loop_distance;
	int period_ms;
	high_resolution_clock::time_point open_loop_t0;
	int64_t last_peroid = timestampf();
	int fail_cnt = 0;
	Vector3f offset_body;
	while (true)
	{
		//only read telemetry data once every iteration
		position_velocity_ned = telemetry->position_velocity_ned();
		euler_angle = telemetry->attitude_euler_angle();
		memcpy(&position_ned, &position_velocity_ned.position, sizeof position_ned);
		memcpy(&velocity_ned, &position_velocity_ned.velocity, sizeof velocity_ned);
		memcpy(&attitude, &euler_angle, sizeof attitude);
		velocity_body = velocityNED2Body(velocity_ned, attitude);

		//receive ground control command
		if (latest<MissionCommand>(mission_command_topic, command_timestamp, command, mission_command_mtx))
		{
			clear<MissionCommand>(mission_command_topic, mission_command_mtx);
			status = command.index;
			param = command.argv[0];
		}
		else
		{
			param = 0.0;
		}

		if (-position_ned.down_m > ALTITUDE_UPPER_BOUND)
		{
			if (status != FORCE_QUIT_COMMAND)
				status = SAFE_QUIT_COMMAND;
		}

		//ignore compute time, this loop update every 20ms
		update<int16_t>(control_status_topic, status, control_status_mtx);
		switch (status)
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
			quitOffboard(offboard);
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
			if (latest<DetectionResult>(target_topic, target_timestamp, target, target_mtx) && timestampf() - target_timestamp < 30)
			{
				remotePrint("VISION CONTROL!");
				status = VISION_CONTROL_MODE;
				vision_roll_thrust_control.angleOffset(roll_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
				//status = VISION_CONTROL_MODE_YAW;
				// vision_roll_thrust_control.angleOffset_Yaw(yaw_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
				// vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust_n, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, -2.0f, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
				fail_cnt = 0;
			}
			else
			{
				thrust = altitude_thrust_control.down(pos_sp_z, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {0.0f, 0.0f, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
			}
			break;
		case VISION_CONTROL_MODE_YAW:
			if (latest<DetectionResult>(target_topic, target_timestamp, target, target_mtx) && timestampf() - target_timestamp < 30)
			{
				//vision_roll_thrust_control.angleOffset(roll_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
				vision_roll_thrust_control.angleOffset_Yaw(yaw_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
				vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust_n, position_ned, velocity_body, attitude, period_ms);
				fail_cnt = 0;
				vision_roll_thrust_control.update_thr_ring_flag(target);
			}
			else
			{
				//vision_roll_thrust_control.hold(roll_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				vision_roll_thrust_control.hold_yaw(thrust, position_ned, velocity_body, attitude, period_ms);
				vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust_n, position_ned, velocity_body, attitude, period_ms);
				fail_cnt++;
			}
			if (fail_cnt > VISION_FAIL_TOLERENCE) //1s?
			{
				vision_roll_thrust_control.can_through_ring_flag = false;
				if (vision_roll_thrust_control.can_through_ring_flag)
				{
					status = VISION_OPEN_LOOP_MODE;
					cout << "VISION OPEN LOOP!" << endl;
					break;
				}
				else
				{
					remotePrint("TIMEOUT!");
					//vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, pos_ned, vel_body, attitude, period_ms);
					//status = BRAKING;
					status = SAFE_QUIT_COMMAND;
					//status = SAFE_QUIT_COMMAND;
					break;
				}
			}
			cout << "yaw_deg_sp: " << yaw_deg << endl;
			input_attitude = {roll_deg, -2.0f, yaw_deg, thrust};
			offbCtrlAttitude(offboard, input_attitude);
			break;
		case VISION_CONTROL_MODE:
			if (latest<DetectionResult>(target_topic, target_timestamp, target, target_mtx) && timestampf() - target_timestamp < 30)
			{
				vision_roll_thrust_control.angleOffset(roll_deg, thrust, target, position_ned, velocity_body, attitude, period_ms);
				fail_cnt = 0;
				vision_roll_thrust_control.update_thr_ring_flag(target);
			}
			else
			{
				vision_roll_thrust_control.hold(roll_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				fail_cnt++;
			}
			if (fail_cnt > VISION_FAIL_TOLERENCE) //1s?
			{
				if (vision_roll_thrust_control.can_through_ring_flag)
				{
					status = VISION_OPEN_LOOP_MODE;
					cout << "VISION OPEN LOOP!" << endl;
					break;
				}
				else
				{
					remotePrint("TIMEOUT!");
					//vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, pos_ned, vel_body, attitude, period_ms);
					//status = BRAKING;
					status = SAFE_QUIT_COMMAND;
					//status = SAFE_QUIT_COMMAND;
					break;
				}
			}
			input_attitude = {roll_deg, -1.0f, yaw_deg, thrust};
			offbCtrlAttitude(offboard, input_attitude);
			break;
		case VISION_OPEN_LOOP_MODE:
			fail_cnt++;
			if (fail_cnt < 1.5 * VISION_FAIL_TOLERENCE)
			{
				vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, -1.0f, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
			}
			else
			{
				vision_roll_thrust_control.braking(roll_deg, pitch_deg, thrust, position_ned, velocity_body, attitude, period_ms);
				input_attitude = {roll_deg, pitch_deg, yaw_deg, thrust};
				offbCtrlAttitude(offboard, input_attitude);
				if (fail_cnt > 2 * VISION_FAIL_TOLERENCE)
				{
					//status = SEARCH_RING;
					status = SAFE_QUIT_COMMAND;
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
#endif