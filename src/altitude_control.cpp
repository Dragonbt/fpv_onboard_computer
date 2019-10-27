#include "control_node.hpp"
#include "protocol.hpp"

float Kp_z, Ki_z, Kd_z;
int SampleTime = 20;
int Times = 0;
bool landing_flag = 0;
float limit_pos_z = -4.0, mid_thrust = 0.55, _int_pos_z = 0.0f;

float altitudeThrustControl( float _pos_sp_z, shared_ptr<Telemetry> telemetry, float dt )
{
    float thrust = 0.0;
    Telemetry::PositionVelocityNED position_velocity_ned = telemetry->position_velocity_ned();
    float _pos_z = position_velocity_ned.position.down_m;
    float _vel_z = position_velocity_ned.velocity.down_m_s;
    if (_pos_z < limit_pos_z || landing_flag) {
		cout << "---Reject PID control,Start land---" << endl;
		if (Times < 2000 / SampleTime){// first 2 second
            thrust = mid_thrust - 0.06f;
		}
		else if (Times < 4000 / SampleTime) {
            thrust = mid_thrust - 0.12f;
		}
		else {
            thrust = mid_thrust - 0.2f;
		}
		Times++;
		landing_flag = 1;
        //cout << "pos_z: " << _pos_z << endl;
	}
    else{
		pushReference(_pos_sp_z);
        float err_pos_z = _pos_sp_z - _pos_z;
		float dif_pos_z = _vel_z;
		float thrust_desired_D = Kp_z * err_pos_z + Kd_z * (-dif_pos_z) + _int_pos_z
			- mid_thrust;
		// The Thrust limits are negated and swapped due to NED-frame.
		//float uMax = -0.06f;
		//float uMin = -1.0f;
		// New limits for the experimental period
		float uMax = -mid_thrust + 0.1f;
		float uMin = -mid_thrust - 0.1f;
		bool stop_integral_D = (thrust_desired_D >= uMax && err_pos_z >= 0.0f) ||
			(thrust_desired_D <= uMin && err_pos_z <= 0.0f);

		if (!stop_integral_D) {
			_int_pos_z += _int_pos_z * Ki_z * dt;

			// limit thrust integral
			int sign_thr_int_z = _int_pos_z > 0 ? 1 : -1;

			_int_pos_z = std::min(fabsf(_int_pos_z), 1.0f) * sign_thr_int_z;
		}
		thrust = thrust_desired_D > uMin ? (thrust_desired_D < uMax ? thrust_desired_D : uMax) : uMin;
		thrust = -thrust;
		//cout << "pos_sp_z: " << _pos_sp_z << " pos_z: " << _pos_z << " vel_z: " << _vel_z << " _int_pos_z: " << _int_pos_z << " thrust: " << thrust << endl;
    }
    return thrust;
}

void altitudeTest( shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, float _Kp_z, float _Ki_z, float _Kd_z )
{
	int status = -3;
	double param = 0;
    Telemetry::EulerAngle euler_angle;
    Offboard::Attitude attitude;
	Telemetry::PositionVelocityNED position_velocity_ned;
    float time_change = 0;
	float _pos_sp_z = 0;
	float yaw = 0;
	float thrust = 0;
	float step = 0;
	Kp_z = _Kp_z;
	Ki_z = _Ki_z;
	Kd_z = _Kd_z;
	cout << "Kp_z:" << Kp_z << endl;
	cout << "Ki_z:" << Ki_z << endl;
	cout << "Kd_z:" << Kd_z << endl;
    high_resolution_clock::time_point t0 = high_resolution_clock::now();

    while(true)
    {
        mission_command_mutex.lock();
        if( ! mission_command_topic.empty() )
		{
			status = mission_command_topic.back().index;
			param = mission_command_topic.back().argv[0];
			mission_command_topic.clear();
		}
        mission_command_mutex.unlock();
		switch( status )
		{
			case -3:
				this_thread::sleep_for(milliseconds(100));
				break;
			case SAFE_QUIT_COMMAND:
				landing_flag = 1;
				status = 1;
				remotePrint(string("auto landing"));
				break;
			case FORCE_QUIT_COMMAND:
				quitOffboard( offboard );
				remotePrint(string("quit mission"));
				return;
			case STEP_COMMAND:
				step = param * -1;
				position_velocity_ned = telemetry->position_velocity_ned();
				_pos_sp_z = position_velocity_ned.position.down_m + step;
				euler_angle = telemetry->attitude_euler_angle();
				yaw = euler_angle.yaw_deg;
				t0 = high_resolution_clock::now();
				thrust = altitudeThrustControl(_pos_sp_z, telemetry, SampleTime );
				attitude = {0.0f, 0.0f, yaw, thrust};
				offbCtrlAttitude(offboard, attitude);
				status = 1;
				remotePrint(string("step response"));
				break;
			case 1:
				time_change = intervalMs( high_resolution_clock::now(), t0);
				if( time_change < SampleTime )
					break;
				t0 = high_resolution_clock::now();
				//control code
				//cout << "Kp_z: " << Kp_z << " Ki_z: "  << Ki_z << " Kd_z: " << Kd_z << endl; 
				thrust = altitudeThrustControl(_pos_sp_z, telemetry, time_change );
				attitude = {0.0f, 0.0f, yaw, thrust};
				offbCtrlAttitude(offboard, attitude);
				break;
		}
	}
    return;
}

/*
void altitude(shared_ptr<Telemetry> telemetry,shared_ptr<Offboard> offboard, double dt, float P, float I, float D) {
    pushReference(_pos_sp_z);
    float Kp_z = P, Ki_z = I, Kd_z = D;
	Offboard::Attitude attitude;
    position = telemetry->position_velocity_ned();
	float _pos_z = position.position.down_m;
	float _vel_z = position.velocity.down_m_s;
	float thrust ;
    //flag_project = 1;
	//protect while height = 2m
    cout << Kp_z << " " << Kd_z << endl;
    cout << _pos_z << " " << limit_pos_z << " " << flag_project << endl;
	if (_pos_z < limit_pos_z || flag_project) {
		cout << "---Reject PID control,Start land---" << endl;
		if (Times < 2000 / SampleTime){// first 2 second
			attitude = { 0.0f, 0.0f, yaw, mid_thrust - 0.02f };
		}
		else if (Times < 4000 / SampleTime) {
			attitude = { 0.0f, 0.0f, yaw, mid_thrust - 0.08f };
		}
		else {
			attitude = { 0.0f, 0.0f, yaw, mid_thrust - 0.2f };
		}
		Times++;
		flag_project = 1;
        cout << "pos_z: " << _pos_z << endl;
	}

	//single-loop PID
	else {
		//start PID control
		cout << "---START PID control---" << endl;
		float err_pos_z = _pos_sp_z - _pos_z;
		float dif_pos_z = _vel_z;
		float thrust_desired_D = Kp_z * err_pos_z + Kd_z * (-dif_pos_z) + _int_pos_z
			- mid_thrust;
		// The Thrust limits are negated and swapped due to NED-frame.
		//float uMax = -0.06f;
		//float uMin = -1.0f;
		// New limits for the experimental period
		float uMax = -mid_thrust + 0.1f;
		float uMin = -mid_thrust - 0.1f;
		bool stop_integral_D = (thrust_desired_D >= uMax && err_pos_z >= 0.0f) ||
			(thrust_desired_D <= uMin && err_pos_z <= 0.0f);

		if (!stop_integral_D) {
			_int_pos_z += _int_pos_z * Ki_z * dt;

			// limit thrust integral
			int sign_thr_int_z = _int_pos_z > 0 ? 1 : -1;

			_int_pos_z = std::min(fabsf(_int_pos_z), 1.0f) * sign_thr_int_z;
		}
		thrust = thrust_desired_D > uMin ? (thrust_desired_D < uMax ? thrust_desired_D : uMax) : uMin;
		thrust = -thrust;
		cout << "pos_sp_z: " << _pos_sp_z << " pos_z: " << _pos_z << " vel_z: " << _vel_z << " _int_pos_z: " << _int_pos_z << " thrust: " << thrust << endl;
		attitude = { 0.0f, 0.0f, yaw, (float)thrust };
	}
	offbCtrlAttitude(offboard, attitude);
	//pushInputAttitude(attitude);
}
*/