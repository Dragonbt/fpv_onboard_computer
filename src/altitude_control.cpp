#include "control_node.hpp"
#include "protocol.hpp"

using namespace std;

vector<float> _int_pos_xy={0.0f, 0.0f};
float Kp_z, Ki_z, Kd_z,Kp_x, Ki_x, Kd_x,Kp_y, Ki_y, Kd_y;
int SampleTime = 20;
int Times = 0;
bool landing_flag = 0;
float limit_pos_z = -4.0, mid_thrust = 0.55, _int_pos_z = 0.0f;
const float P_I = 3.14159265f;
const float offset_pitch = 0.033f;//only use when x&y is open-loop
const float offset_roll = -0.02f;//only use when x&y is open-loop
int time_loop = 0;
float tilt_max = P_I / 4;
float offset_thrust;//offset lift attenuation due to tilting
bool flow_effective = true, vision_effective = false, position_hold_xy = true, through_ring = false;

bool alt_with_laser=true;
float my_pos_sp_x, my_pos_sp_y,my_pos_sp_z;
int64_t time_ms = 0;

float altitudeThrustControl( float _pos_sp_z, shared_ptr<Telemetry> telemetry, float dt )
{
    float thrust = 0.0;
    Telemetry::PositionVelocityNED position_velocity_ned = telemetry->position_velocity_ned();
	Telemetry::EulerAngle euler_angle = telemetry->attitude_euler_angle();;
    float _pos_z = position_velocity_ned.position.down_m;
    float _vel_z = position_velocity_ned.velocity.down_m_s;
	float _pitch = euler_angle.pitch_deg;
	float _roll = euler_angle.roll_deg;
	offset_thrust = mid_thrust / (cos(_pitch * P_I / 180)*cos(_roll*P_I / 180));
	//cout << "pitch=" << _pitch << "roll=" << _roll << endl;
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
        float err_pos_z;
		if (alt_with_laser) err_pos_z = _pos_sp_z - _pos_z;
		else err_pos_z = _pos_sp_z;//alt_with_vision;
		float dif_pos_z = _vel_z;
		float thrust_desired_D = Kp_z * err_pos_z + Kd_z * (-dif_pos_z) + _int_pos_z
			- offset_thrust;
		// The Thrust limits are negated and swapped due to NED-frame.
		//float uMax = -0.06f;
		//float uMin = -1.0f;
		// New limits for the experimental period
		float uMax = -mid_thrust + 0.25f;
		float uMin = -mid_thrust - 0.25f;
		bool stop_integral_D = (thrust_desired_D >= uMax && err_pos_z >= 0.0f) ||
			(thrust_desired_D <= uMin && err_pos_z <= 0.0f);

		if (!stop_integral_D) {
			_int_pos_z += err_pos_z * Ki_z * dt;

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

float lengthofvector(vector<float> a) {
	float sum = 0;
	for (size_t i = 0; i < a.size(); i++) {
		sum += a[i] * a[i];
	}
	return (float)sqrt(sum);
}

vector<float> operator+(vector<float>a, vector<float>b) {
	vector<float> res;
	for (size_t i = 0; i < a.size(); i++) {
		res.push_back(a[i] + b[i]);
	}
	return res;
}

vector<float> operator-(vector<float>a, vector<float>b) {
	vector<float> res;
	for (size_t i = 0; i < a.size(); i++) {
		res.push_back(a[i] - b[i]);
	}
	return res;
}

vector<float> operator*(vector<float>a, vector<float>b) {
	vector<float> res;
	for (size_t i = 0; i < a.size(); i++) {
		res.push_back(a[i] * b[i]);
	}
	return res;
}

vector<float> operator*(vector<float>a, float b) {
	vector<float> res;
	for (size_t i = 0; i < a.size(); i++) {
		res.push_back(a[i] * b);
	}
	return res;
}

vector<float> operator*(float a, vector<float> b) {
	vector<float> res;
	for (size_t i = 0; i < b.size(); i++) {
		res.push_back(b[i] * a);
	}
	return res;
}

vector<float> positionThrustControl(vector<float> _pos_sp, shared_ptr<Telemetry> telemetry, float dt) {
	float alt_thrust = altitudeThrustControl(_pos_sp[2], telemetry, dt);
	vector<float> _pos_err = { 0.0f,0.0f };
	vector<float> _thr_sp = { 0.0f,0.0f,alt_thrust };
	vector<float> _pos = { 0.0f,0.0f };
	vector<float> _vel = { 0.0f,0.0f };
	vector<float> _vel_err = { 0.0f,0.0f };
	vector<float> Kp_xy = { Kp_x,Kp_y };
	vector<float> Ki_xy = { Ki_x,Ki_y };
	vector<float> Kd_xy = { Kd_x,Kd_y };
	
	float thrust_max = 1.0f;
	Telemetry::PositionVelocityNED position_velocity_ned = telemetry->position_velocity_ned();
	Telemetry::EulerAngle euler_angle = telemetry->attitude_euler_angle();
	float yaw = euler_angle.yaw_deg * P_I / 180;
	//float yaw = 45 * P_I / 180;
	float _pos_err_n, _pos_err_e;
	if (flow_effective) {
		_pos[0] = position_velocity_ned.position.north_m;
		_pos[1] = position_velocity_ned.position.east_m;
		_pos_err_n = _pos_sp[0] - _pos[0];
		_pos_err_e = _pos_sp[1] - _pos[1];
		//_pos_err_n = 1.0f;
		//_pos_err_e = 1.0f;
		_pos_err[0] = _pos_err_n * cos(yaw) + _pos_err_e * sin(yaw);
		_pos_err[1] = -1.0f * _pos_err_n * sin(yaw) + _pos_err_e * cos(yaw);
		//cout << "_pos_err_x:" << _pos_err[0] << endl;
		//cout << "_pos_err_y:" << _pos_err[1] << endl;
	}
	else if (vision_effective) {
		float _yaw = euler_angle.yaw_deg * P_I / 180;
		_pos[0] = position_velocity_ned.position.north_m;
		_pos[1] = position_velocity_ned.position.east_m;
		float _pos_err_n = my_pos_sp_x - _pos[0];
		float _pos_err_e = my_pos_sp_y - _pos[1];
		float _pos_err_x = _pos_err_n * cos(_yaw) + _pos_err_e * sin(_yaw);
		//float _pos_err_y = -_pos_err_n * sin(_yaw) + _pos_err_e * cos(_yaw);
		//_pos_err[0] = _pos_sp[0];
		_pos_err[0] = _pos_err_x;
		_pos_err[1] = _pos_sp[1];
		cout << "_pos_err_x_from_flow = " << my_pos_sp_x << " " << "_pos_err_x_from_vision = " << _pos_sp[0] << endl;

	}
	if (position_hold_xy) {//state=take_off||state=landing
		_vel[0] = position_velocity_ned.velocity.north_m_s;
		_vel[1] = position_velocity_ned.velocity.east_m_s;
		_vel_err[0] = -_vel[0];
		_vel_err[1] = -_vel[1];
		vector<float> thrust_desired_NE = {0.0f,0.0f};
		thrust_desired_NE = Kp_xy * _pos_err + Kd_xy * _vel_err + _int_pos_xy;
		float thrust_max_NE_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
		float thrust_max_NE = sqrtf(thrust_max * thrust_max - _thr_sp[2] * _thr_sp[2]);
		thrust_max_NE = thrust_max_NE_tilt < thrust_max_NE ? thrust_max_NE_tilt : thrust_max_NE;
		// Saturate thrust in NE-direction.
		_thr_sp[0] = thrust_desired_NE[0];
		_thr_sp[1] = thrust_desired_NE[1];

		if ((thrust_desired_NE[0] * thrust_desired_NE[0] + thrust_desired_NE[1] * thrust_desired_NE[1]) > thrust_max_NE * thrust_max_NE) {
			float mag = lengthofvector(thrust_desired_NE);
			_thr_sp[0] = thrust_desired_NE[0] / mag * thrust_max_NE;
			_thr_sp[1] = thrust_desired_NE[1] / mag * thrust_max_NE;
		}
		// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
		// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
		float arw_gain = 2.f / Kp_xy[0];

		vector<float> vel_err_lim = { 0.0f,0.0f };
		vel_err_lim = _pos_err - (thrust_desired_NE - _thr_sp) * arw_gain;
		

		// Update integral
		_int_pos_xy = _int_pos_xy + Ki_xy * vel_err_lim * dt;
		//_thr_int(1) += _param_mpc_xy_vel_i.get() * vel_err_lim(1) * dt;
	}
	else if (through_ring) {//state = through
		//float _pitch = euler_angle.pitch_deg;
		//float _roll = euler_angle.roll_deg;
		//float thrust_z = alt_thrust * cos(_pitch) * cos(_roll);
		//to be continued
	}
	return _thr_sp;
}

void altitudeTest( shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, float _Kp_z, float _Ki_z, float _Kd_z,float _Kp_x, float _Ki_x, float _Kd_x,float _Kp_y, float _Ki_y, float _Kd_y )
{
	int status = -3;
	double param = 0;
    Telemetry::EulerAngle euler_angle;
    Offboard::Attitude attitude;
	Telemetry::PositionVelocityNED position_velocity_ned;
    float time_change = 0;
	float _pos_sp_z = 0;
	vector<float> _pos_sp = {0.0f, 0.0f, 0.0f};
	float yaw = 0;
	float thrust = 0;
	vector<float> _thr_sp={0.0f,0.0f,0.0f};
	float step = 0;
	float _roll_sp, _pitch_sp;
	float len;
	float off_x, off_y, off_z, off_n, off_e, off_d;
	vector<float> att_sp = {0.0f, 0.0f};

	Kp_z = _Kp_z;
	Ki_z = _Ki_z;
	Kd_z = _Kd_z;

	Kp_x = _Kp_x;
	Ki_x = _Ki_x;
	Kd_x = _Kd_x;

	Kp_y = _Kp_y;
	Ki_y = _Ki_y;
	Kd_y = _Kd_y;
	cout << "Kp_z:" << Kp_z << endl;
	cout << "Ki_z:" << Ki_z << endl;
	cout << "Kd_z:" << Kd_z << endl;

	cout << "Kp_x:" << Kp_x << endl;
	cout << "Ki_x:" << Ki_x << endl;
	cout << "Kd_x:" << Kd_x << endl;

	cout << "Kp_y:" << Kp_y << endl;
	cout << "Ki_y:" << Ki_y << endl;
	cout << "Kd_y:" << Kd_y << endl;
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
	DetectionResult target;
    while(true)
    {
        mission_command_mutex.lock();
        if( ! mission_command_topic.empty() )
		{
			status = mission_command_topic.back().index;
			param = mission_command_topic.back().argv[0];
			mission_command_topic.clear();
		}
		else{
			param = 0.0;
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
				attitude = {0.0f, 0.0f, 0.0f, thrust};
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
				//attitude = { 5*offset_roll*180/P_I, 5*offset_pitch*180/P_I, yaw, thrust };
				attitude = { 0.0f, 0.0f, 0.0f, thrust };
				/*
				time_loop++;
				if(time_loop < 5) {
					attitude = { 0.0f, 0.0f, yaw, thrust};
					cout << "forward" << endl;
				}
				else if(time_loop < 10) {
					attitude = { 2*offset_roll*180/P_I, 3*offset_pitch*180/P_I, yaw, thrust };
					cout << "back" << endl;
				}
				else {
					time_loop = 0;
					attitude = { 0.0f, 0.0f, yaw, thrust };
				}
				*/
				offbCtrlAttitude(offboard, attitude);
				//attitude = {0.0, 10.0, 0.0, 0.2};
				//offbCtrlAttitude(offboard, attitude);
				break;
			case FLOW_HOLD_COMMAND:
				flow_effective = true;
				vision_effective = false;
				alt_with_laser = true;
				remotePrint(string("FLOW HOLD"));
				cout << param << endl;
				position_velocity_ned = telemetry->position_velocity_ned();
				_pos_sp[0] = position_velocity_ned.position.north_m + param;
				_pos_sp[1] = position_velocity_ned.position.east_m;
				_pos_sp[2] = position_velocity_ned.position.down_m;
				my_pos_sp_x = _pos_sp[0];
				my_pos_sp_y = _pos_sp[1];
				my_pos_sp_z = _pos_sp[2];
				euler_angle = telemetry->attitude_euler_angle();
				yaw = euler_angle.yaw_deg;
				//yaw = 0.0f;
				t0 = high_resolution_clock::now();
				_thr_sp = positionThrustControl(_pos_sp, telemetry, SampleTime );
				_thr_sp[1] = 0.707f * _thr_sp[2] > _thr_sp[1] ? (-0.707f * _thr_sp[2] < _thr_sp[1] ? _thr_sp[1] : -0.707f * _thr_sp[2]) : 0.707f * _thr_sp[2];
				_thr_sp[0] = 0.707f * _thr_sp[2] > _thr_sp[0] ? (-0.707f * _thr_sp[2] < _thr_sp[0] ? _thr_sp[0] : -0.707f * _thr_sp[2]) : 0.707f * _thr_sp[2];
				//cout << "_thr_sp[0] = " << _thr_sp[0] << " " << "_thr_sp[1] = " << _thr_sp[1] << " " << "_thr_sp[2] = " << _thr_sp[2] << endl;
				_roll_sp = asinf(_thr_sp[1] / _thr_sp[2]);
				
				_pitch_sp = -asinf(_thr_sp[0] / _thr_sp[2]);
				att_sp = {asinf(_thr_sp[1] / _thr_sp[2]),-asinf(_thr_sp[0] / _thr_sp[2])};
				if((_roll_sp * _roll_sp + _pitch_sp * _pitch_sp) > tilt_max * tilt_max){
					
					len = lengthofvector(att_sp);
					_roll_sp = att_sp[0] / len * tilt_max;
					_pitch_sp = att_sp[1] / len * tilt_max;
				}
				

				//cout << "roll" << asinf(_thr_sp[1] / _thr_sp[2]) << endl;
				//cout << "pitch" << asinf(_thr_sp[0] / _thr_sp[2]) << endl;
				attitude = {rad2deg(_roll_sp), rad2deg(_pitch_sp), yaw, _thr_sp[2]};
				cout << attitude << endl;
				offbCtrlAttitude(offboard, attitude);
				status = 3;
				break;
			case 3:
				time_change = intervalMs( high_resolution_clock::now(), t0);
				if( time_change < SampleTime )
					break;
				t0 = high_resolution_clock::now();
				_thr_sp = positionThrustControl(_pos_sp, telemetry, SampleTime );
				_thr_sp[1] = 0.707f * _thr_sp[2] > _thr_sp[1] ? (-0.707f * _thr_sp[2] < _thr_sp[1] ? _thr_sp[1] : -0.707f * _thr_sp[2]) : 0.707f * _thr_sp[2];
				_thr_sp[0] = 0.707f * _thr_sp[2] > _thr_sp[0] ? (-0.707f * _thr_sp[2] < _thr_sp[0] ? _thr_sp[0] : -0.707f * _thr_sp[2]) : 0.707f * _thr_sp[2];
				//cout << "_thr_sp[0] = " << _thr_sp[0] << " " << "_thr_sp[1] = " << _thr_sp[1] << " " << "_thr_sp[2] = " << _thr_sp[2] << endl;
				_roll_sp = asinf(_thr_sp[1] / _thr_sp[2]);
				
				_pitch_sp = -asinf(_thr_sp[0] / _thr_sp[2]);
				att_sp = {asinf(_thr_sp[1] / _thr_sp[2]),-asinf(_thr_sp[0] / _thr_sp[2])};
				if((_roll_sp * _roll_sp + _pitch_sp * _pitch_sp) > tilt_max * tilt_max){
					
					len = lengthofvector(att_sp);
					_roll_sp = att_sp[0] / len * tilt_max;
					_pitch_sp = att_sp[1] / len * tilt_max;
				}
				

				//cout << "roll" << asinf(_thr_sp[1] / _thr_sp[2]) << endl;
				//cout << "pitch" << asinf(_thr_sp[0] / _thr_sp[2]) << endl;
				attitude = {rad2deg(_roll_sp), rad2deg(_pitch_sp), yaw, _thr_sp[2]};
				cout << attitude << endl;
				offbCtrlAttitude(offboard, attitude);
				break;
			case VISION_HOLD_COMMAND:
				time_ms = intervalMs(high_resolution_clock::now(), init_timepoint);
				flow_effective = true;
				vision_effective = false;
				alt_with_laser = true;
				remotePrint(string("VISION HOLD"));
				position_velocity_ned = telemetry->position_velocity_ned();
				target_mutex.lock();
				if(target_topic.empty())
				{
					target.confidence = -1;
				}
				else{
					target = target_topic.back();
				}
				target_mutex.unlock();
				if( target.confidence > 0 && time_ms - target.time_ms < 1000)
				{
					euler_angle = telemetry->attitude_euler_angle();
					yaw = euler_angle.yaw_deg * P_I / 180;
					off_x = target.z_m;
					off_y = target.x_m;
					off_z = target.y_m;
					off_n = off_x * cos(yaw) - off_y * sin(yaw);
					off_e = off_x * sin(yaw) + off_y * cos(yaw);
					off_d = off_z;
					_pos_sp[0] = my_pos_sp_x;
					_pos_sp[1] = position_velocity_ned.position.east_m + off_e;
					_pos_sp[2] = position_velocity_ned.position.down_m + off_d;
				}
				else if(time_ms - target.time_ms > 1000){
					remotePrint(string("TIME OUT!"));
					status = FLOW_HOLD_COMMAND;					
				}
				time_change = intervalMs( high_resolution_clock::now(), t0);
				if( time_change < SampleTime )
					break;
				t0 = high_resolution_clock::now();
				_thr_sp = positionThrustControl(_pos_sp, telemetry, SampleTime );
				_thr_sp[1] = 0.707f * _thr_sp[2] > _thr_sp[1] ? (-0.707f * _thr_sp[2] < _thr_sp[1] ? _thr_sp[1] : -0.707f * _thr_sp[2]) : 0.707f * _thr_sp[2];
				_thr_sp[0] = 0.707f * _thr_sp[2] > _thr_sp[0] ? (-0.707f * _thr_sp[2] < _thr_sp[0] ? _thr_sp[0] : -0.707f * _thr_sp[2]) : 0.707f * _thr_sp[2];
				//cout << "_thr_sp[0] = " << _thr_sp[0] << " " << "_thr_sp[1] = " << _thr_sp[1] << " " << "_thr_sp[2] = " << _thr_sp[2] << endl;
				_roll_sp = asinf(_thr_sp[1] / _thr_sp[2]);
				
				_pitch_sp = -asinf(_thr_sp[0] / _thr_sp[2]);
				att_sp = {asinf(_thr_sp[1] / _thr_sp[2]),-asinf(_thr_sp[0] / _thr_sp[2])};
				if((_roll_sp * _roll_sp + _pitch_sp * _pitch_sp) > tilt_max * tilt_max){
					
					len = lengthofvector(att_sp);
					_roll_sp = att_sp[0] / len * tilt_max;
					_pitch_sp = att_sp[1] / len * tilt_max;
				}
				

				//cout << "roll" << asinf(_thr_sp[1] / _thr_sp[2]) << endl;
				//cout << "pitch" << asinf(_thr_sp[0] / _thr_sp[2]) << endl;
				attitude = {rad2deg(_roll_sp), rad2deg(_pitch_sp), yaw, _thr_sp[2]};
				cout << attitude << endl;
				offbCtrlAttitude(offboard, attitude);
				break;
			/*
			case VISION_HOLD_COMMAND:
				time_ms = intervalMs(high_resolution_clock::now(), init_timepoint);
				remotePrint(string("VISION HOLD"));
				flow_effective = false;
				vision_effective = true;
				alt_with_laser = false;
				target_mutex.lock();
				if(target_topic.empty())
				{
					target.confidence = -1;
				}
				else{
					target = target_topic.back();
				}
				target_mutex.unlock();
				if( target.confidence > 0 && time_ms - target.time_ms < 100)
				{
					_pos_sp[0] = target.z_m;
					_pos_sp[1] = target.x_m;
					_pos_sp[2] = target.y_m;
				}
				else{
					status = FLOW_HOLD_COMMAND;
					param = 0;
					remotePrint(string("VISION FAIL!"));
					cout << "[WARNING]: " << "VISION FAIL " << target.confidence << " " << time_ms - target.time_ms << endl;
					break;
				}
				

				time_change = intervalMs( high_resolution_clock::now(), t0);
				if( time_change < SampleTime )
					break;
				t0 = high_resolution_clock::now();
				_thr_sp = positionThrustControl(_pos_sp, telemetry, SampleTime );
				_thr_sp[1] = 0.707f * _thr_sp[2] > _thr_sp[1] ? (-0.707f * _thr_sp[2] < _thr_sp[1] ? _thr_sp[1] : -0.707f * _thr_sp[2]) : 0.707f * _thr_sp[2];
				_thr_sp[0] = 0.707f * _thr_sp[2] > _thr_sp[0] ? (-0.707f * _thr_sp[2] < _thr_sp[0] ? _thr_sp[0] : -0.707f * _thr_sp[2]) : 0.707f * _thr_sp[2];
				//cout << "_thr_sp[0] = " << _thr_sp[0] << " " << "_thr_sp[1] = " << _thr_sp[1] << " " << "_thr_sp[2] = " << _thr_sp[2] << endl;
				_roll_sp = asinf(_thr_sp[1] / _thr_sp[2]);
				
				_pitch_sp = -asinf(_thr_sp[0] / _thr_sp[2]);
				att_sp = {asinf(_thr_sp[1] / _thr_sp[2]),-asinf(_thr_sp[0] / _thr_sp[2])};
				if((_roll_sp * _roll_sp + _pitch_sp * _pitch_sp) > tilt_max * tilt_max){
					
					len = lengthofvector(att_sp);
					_roll_sp = att_sp[0] / len * tilt_max;
					_pitch_sp = att_sp[1] / len * tilt_max;
				}
				

				//cout << "roll" << asinf(_thr_sp[1] / _thr_sp[2]) << endl;
				//cout << "pitch" << asinf(_thr_sp[0] / _thr_sp[2]) << endl;
				if( param != 0)
				{
					yaw = yaw + param;
					param = 0;
				}
				cout << yaw << endl;
				attitude = {rad2deg(_roll_sp), rad2deg(_pitch_sp), yaw, _thr_sp[2]};
				cout << attitude << endl;
				offbCtrlAttitude(offboard, attitude);
				break;
			*/
		}
	}
    return;
}

float rad2deg(float rad)
{
	return rad * 180 / P_I;
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