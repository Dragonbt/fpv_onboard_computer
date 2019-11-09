#include "control_sync.hpp"

void FlowPosThrustControl::reset(FileNode flow_pid, FileNode altitude_pid) {
	flow_pid["Kp_x"] >> Kp_x;
	flow_pid["Ki_x"] >> Ki_x;
	flow_pid["Kd_x"] >> Kd_x;
	flow_pid["Kp_y"] >> Kp_y;
	flow_pid["Ki_y"] >> Ki_y;
	flow_pid["Kd_y"] >> Kd_y;
    Kp = {Kp_x, Kp_y};
    Ki = {Ki_x, Ki_y};
    Kd = {Kd_x, Kd_y};
	int_pos_xy = { 0.0f,0.0f };
	altitude_thrust_control.reset(altitude_pid);
    cout << "FLOW PID:" << endl;
    cout << this->Kp_x << endl;
    cout << this->Ki_x << endl;
    cout << this->Kd_x << endl;

    cout << this->Kp_y << endl;
    cout << this->Ki_y << endl;
    cout << this->Kd_y << endl;
	return;
}

void FlowPosThrustControl::positionBodyOffset( float& roll_deg, float& pitch_deg, float& thrust, Vector3f offset_body, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    Vector2f pos_ne = {pos_ned.north_m, pos_ned.east_m};
    pos_err_xy = {offset_body.x, offset_body.y};
    pos_err_ne = xy2ne(pos_err_xy, attitude.yaw_deg);
    pos_sp_ne = pos_ne + pos_err_ne;
    vel_err_xy = {-vel_body.x_m_s, -vel_body.y_m_s};
    alt_thrust = altitude_thrust_control.downOffset(offset_body.z, pos_ned, vel_body, attitude, dt_ms);
    calcRollPitchThrust(roll_deg, pitch_deg, thrust);
}

void FlowPosThrustControl::hold(float& roll_deg, float& pitch_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    Vector2f pos_ne = {pos_ned.north_m, pos_ned.east_m};
    pos_err_ne = pos_sp_ne - pos_ne;
    pos_err_xy = ne2xy(pos_err_ne, attitude.yaw_deg);
    vel_err_xy = {-vel_body.x_m_s, -vel_body.y_m_s};
    alt_thrust = altitude_thrust_control.hold(pos_ned, vel_body, attitude, dt_ms);
    calcRollPitchThrust(roll_deg, pitch_deg, thrust);
}

void FlowPosThrustControl::calcRollPitchThrust(float& roll_deg, float& pitch_deg, float& thrust)
{
    pos_err_xy_topic.update(pos_err_xy);
    ne_reference_topic.update(pos_sp_ne);
    Vector2f thrust_xy = {0.0f, 0.0f};
    Vector2f thrust_desired = {0.0f, 0.0f};
    thrust_desired = Kp * pos_err_xy + Kd * vel_err_xy + int_pos_xy;
    thrust_xy = thrust_desired;

    float thrust_max_NE_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_NE = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_NE = thrust_max_NE_tilt < thrust_max_NE ? thrust_max_NE_tilt : thrust_max_NE;
	// Saturate thrust in NE-direction.

	if ((thrust_xy.x * thrust_xy.x + thrust_xy.y * thrust_xy.y) > thrust_max_NE * thrust_max_NE) {
		float mag = sqrtf(thrust_xy.x * thrust_xy.x + thrust_xy.y * thrust_xy.y);
		thrust_xy = thrust_xy / ( mag / thrust_max_NE);
	}
	// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	float arw_gain = 2.f / Kp.x;

	Vector2f vel_err_lim = { 0.0f,0.0f };
	vel_err_lim = pos_err_xy - (thrust_desired - thrust_xy) * arw_gain;


	// Update integral
	int_pos_xy = int_pos_xy + Ki * vel_err_lim * time_change;
    Vector3f thrust_xyz = {thrust_xy.x, thrust_xy.y, alt_thrust};
    thrust = alt_thrust;
    roll_deg = rad2deg( atan2f(thrust_xyz.y, thrust_xyz.z) );
    pitch_deg = rad2deg( atan2f(-thrust_xyz.x, sqrtf(thrust_xyz.y*thrust_xyz.y+thrust_xyz.z*thrust_xyz.z)) );
    //roll_deg = rad2deg( asinf(thrust_xyz.y / thrust_xyz.z) );
	//pitch_deg = rad2deg(-asinf(thrust_xyz.x / thrust_xyz.z) );

    roll_deg=limit_values(roll_deg,-10.0f,10.0f);
    pitch_deg= limit_values(pitch_deg,-10.0f,10.0f);
}

void VisionRollThrustControl::reset(FileNode vision_pid, FileNode altitude_pid)
{
    vision_pid["Ky"] >> Ky;
    vision_pid["Kz"] >> Kz;
    vision_pid["Kp_y"] >> Kp_y;
    vision_pid["Ki_y"] >> Ki_y;
    vision_pid["Kd_y"] >> Kd_y;

    altitude_thrust_control.reset(altitude_pid);

    cout << "Vision Gain:" << endl;
    cout << this->Ky << endl;
    cout << this->Kz << endl;

    cout << "Vision PID:" << endl;
    cout << this->Kp_y << endl;
    cout << this->Ki_y << endl;
    cout << this->Kd_y << endl;
}

/*
void VisionRollThrustControl::angleOffset( float& roll_deg, float& thrust, DetectionResult target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;
    float roll_rad = deg2rad( attitude.roll_deg );
    //convert target from camera to body
    float target_y_m = target.x_m * cos( roll_rad ) - target.y_m * sin( roll_rad );
    float target_z_m = target.x_m * sin( roll_rad ) + target.y_m * cos( roll_rad );
    float target_x_m = target.z_m;
    z_deg = rad2deg( atan2f(target_z_m, target_x_m) );
    pos_sp_z = Kz * z_deg + pos_ned.down_m;
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);

    vel_y = vel_body.y_m_s;
    vel_sp_y = vel_body.x_m_s * target_y_m / target_x_m;
    roll_deg = calcRoll();
    thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    limit_values(calcRoll(), -45.0f, 45.0f);
    limit_values(thrust, 0.25f, 0.85f);
    return;
}

void VisionRollThrustControl::hold(float& roll_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    alt_thrust = altitude_thrust_control.hold(pos_ned, vel_body, attitude, dt_ms);
    
    vel_y = vel_body.y_m_s;
    roll_deg = calcRoll();
    thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    limit_values(calcRoll(), -45.0f, 45.0f);
    limit_values(thrust, 0.25f, 0.85f);
    return;
}

float VisionRollThrustControl::calcRoll()
{
    vel_err_y = vel_sp_y - vel_y;
    return Kp_y * vel_err_y;
}
*/
void VisionRollThrustControl::angleOffset( float& roll_deg, float& thrust, DetectionResult target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;
    float roll_rad = deg2rad( attitude.roll_deg );
    //convert target from camera to body
    float target_y_m = target.x_m * cos( roll_rad ) - target.y_m * sin( roll_rad );
    float target_z_m = target.x_m * sin( roll_rad ) + target.y_m * cos( roll_rad );
    float target_x_m = target.z_m;
    z_deg = rad2deg( atan2f(target_z_m, target_x_m) );
    y_deg = rad2deg( atan2f(target_y_m, target.x_m) );

    err_pos_y = Ky * y_deg;
    err_pos_z = Kz * z_deg;
    vel_err = -vel_body.y_m_s;

    pos_sp_z = pos_ned.down_m + err_pos_z;
    alt_thrust = altitude_thrust_control.down(pos_sp_z, pos_ned, vel_body, attitude, dt_ms);

    roll_deg = calcRoll();
	thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
}

void VisionRollThrustControl::hold(float& roll_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    alt_thrust = altitude_thrust_control.hold(pos_ned, vel_body, attitude, dt_ms);
    
    vel_err = -vel_body.y_m_s;
    roll_deg = calcRoll();
    thrust = alt_thrust / ( cos(deg2rad(attitude.roll_deg)) * cos(deg2rad(attitude.pitch_deg)) );
    return;
}

float VisionRollThrustControl::calcRoll()
{
    float thr_sp_y;
	thrust_desired_y = Kp_y * err_pos_y + Kd_y * vel_err + int_pos_y;
	float thrust_max_y_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_y = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_y = thrust_max_y_tilt < thrust_max_y ? thrust_max_y_tilt : thrust_max_y;
	// Saturate thrust in NE-direction.
    thr_sp_y = thrust_desired_y;

	if (thr_sp_y  > thrust_max_y) {
		thr_sp_y = thrust_max_y;			
	}
	// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	float arw_gain = 2.f / Kp_y;
    float vel_err_lim = 0.0f;
	vel_err_lim = err_pos_y - (thrust_desired_y - thr_sp_y) * arw_gain;
	
    // Update integral
	int_pos_y = int_pos_y + Ki_y * vel_err_lim * time_change;
	//_thr_int(1) += _param_mpc_xy_vel_i.get() * vel_err_lim(1) * dt;

	thr_sp_y = limit_values(thr_sp_y, -0.707f * alt_thrust, 0.707f * alt_thrust);
	//cout << "_thr_sp[0] = " << _thr_sp[0] << " " << "_thr_sp[1] = " << _thr_sp[1] << " " << "_thr_sp[2] = " << _thr_sp[2] << endl;
	float roll_sp = asinf(thr_sp_y / alt_thrust);
    return rad2deg(roll_sp);
}

void AltitudeThrustControl::reset(FileNode altitude_pid)
{
    altitude_pid["Kp_z"] >> Kp_z;
    altitude_pid["Ki_z"] >> Ki_z;
    altitude_pid["Kd_z"] >> Kd_z;
    int_pos_z = 0;
    stop_integral_D = false;

    cout << "Altitude PID:" << endl;
    cout << this->Kp_z << endl;
    cout << this->Ki_z << endl;
    cout << this->Kd_z << endl;
    return;
}

float AltitudeThrustControl::landing()
{
    float thrust;
	if (times < 2000 / 20){// first 2 second
        thrust = mid_thrust - 0.06f;
	}
	else if (times < 4000 / 20) {
        thrust = mid_thrust - 0.12f;
	}
	else {
        thrust = mid_thrust - 0.2f;
	}
	times++;
    return thrust;
}

void AltitudeThrustControl::braking(float& roll_deg, float& pitch_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms) {
	float alt_thrust = hold(pos_ned, vel_body, attitude, dt_ms);
	Vector2f Vxy = { vel_body.x_m_s, vel_body.y_m_s};
	Vector2f Vxy_sp = { min_vx_find_loop ,0.0f };
	Vector2f Vxy_err = Vxy_sp - Vxy;
	Vector2f Kp_brank = { 0.1f,0.1f };
	
	//float Ki_brank = 0.01f;
	//float thrustx = Kp_brank * Vx_err + _int_vel_x;
	Vector2f thrustxy = Kp_brank * Vxy_err;
	//_int_vel_x += Ki_brank * dt * Vx_err;

	float thrust_max_tilt = fabsf(alt_thrust) * tanf(tilt_max);//while in take_off or landing state i think the "cos(_pitch) * cos(_roll) = 1" => alt_thrust = thrust_z 
	float thrust_max_xy = sqrtf(thrust_max * thrust_max - alt_thrust * alt_thrust);
	thrust_max_xy = thrust_max_tilt < thrust_max_xy ? thrust_max_tilt : thrust_max_xy;
	// Saturate thrust in NE-direction.
    float mag = mag2f(thrustxy);
	if (mag * mag > thrust_max_xy * thrust_max_xy) {
		thrustxy = thrustxy / (mag / thrust_max_xy);
	}
	//if (Vx_err > 0 && _int_vel_x < 0) _int_vel_x
	float pitch = -atanf(thrustxy.x / alt_thrust);
	float roll = atanf(thrustxy.y / alt_thrust);
	pitch = limit_values(pitch, -P_I / 6.0f, P_I / 6.0f);
	roll = limit_values(roll, -P_I / 6.0f, P_I / 6.0f);
	roll_deg = rad2deg(roll);
	pitch_deg = rad2deg(pitch);
	thrust = alt_thrust;
}

float AltitudeThrustControl::downOffset( float err, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms )
{
    time_change = dt_ms / 1000.0f;
    pos_z = pos_ned.down_m;
    vel_z = vel_body.z_m_s;
    roll = attitude.roll_deg;
    pitch = attitude.pitch_deg;
    pos_sp_z = pos_z + err;
    return calcThrust();
}

float AltitudeThrustControl::down(float pos_sp_z, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    pos_z = pos_ned.down_m;
    vel_z = vel_body.z_m_s;
    roll = attitude.roll_deg;
    pitch = attitude.pitch_deg;
    this->pos_sp_z = pos_sp_z;
    return calcThrust();
}

float AltitudeThrustControl::hold(PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms)
{
    time_change = dt_ms / 1000.0f;
    pos_z = pos_ned.down_m;
    vel_z = vel_body.z_m_s;
    roll = attitude.roll_deg;
    pitch = attitude.pitch_deg;
    return calcThrust();
}

float AltitudeThrustControl::calcThrust()
{
    float thrust;
    down_reference_topic.update(pos_sp_z);
    err_pos_z = pos_sp_z - pos_z;
    //offset_thrust = mid_thrust / (cos(deg2rad(pitch))*cos(deg2rad(roll)));
    offset_thrust = mid_thrust;
    thrust_desired_D = Kp_z * err_pos_z + Kd_z * (-vel_z) + int_pos_z - offset_thrust;
    stop_integral_D = (thrust_desired_D >= uMax && err_pos_z >= 0.0f) ||
			(thrust_desired_D <= uMin && err_pos_z <= 0.0f);
    if( ! stop_integral_D )
    {
        int_pos_z += err_pos_z * Ki_z * time_change;
        sign_thr_int_z = int_pos_z > 0 ? 1 : -1;
		int_pos_z = std::min(fabsf(int_pos_z), 1.0f) * sign_thr_int_z;
    }
    thrust = limit_values(-thrust_desired_D, uMin, uMax);
    return thrust;
}

void healthCheck( shared_ptr<Telemetry> telemetry )
{
    while ( ! telemetry->health_all_ok() ) {
        cout << "[LOGGING]: Waiting for health_all_ok" << endl;
        this_thread::sleep_for(seconds(1));
    }
}

void waitForArmed( shared_ptr<Telemetry> telemetry )
{
    while( ! telemetry->armed() )
    {
        this_thread::sleep_for(seconds(1));
    }
    cout << "[LOGGING]: armed" << endl;
    return;
}

void quitOffboard( shared_ptr<Offboard> offboard )
{
    Offboard::Result offboard_result;
    if( offboard->is_active() ){
        offboard_result = offboard->stop();
        if ( offboard_result != Offboard::Result::SUCCESS ){
            cout << string("[ERROR]: ") + Offboard::result_str(offboard_result) << endl;
            cout << "[ERROR]: stop offboard error" << endl;
            return;
        }
    }
    return;
}

void offbCtrlAttitude(shared_ptr<Offboard> offboard, Offboard::Attitude attitude)
{
    InputAttitude input_attitude;
    Offboard::Result offboard_result;
    if( ! offboard->is_active() ){
        offboard->set_attitude(attitude);
        offboard_result = offboard->start();
        if ( offboard_result != Offboard::Result::SUCCESS ){
            cout << string("[ERROR]: ") + Offboard::result_str(offboard_result) << endl;
            cout << "[ERROR]: unable to start offboard" << endl;
            return;
        }
    }
    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_attitude(attitude);
    input_attitude.roll_deg = attitude.roll_deg;
    input_attitude.pitch_deg = attitude.pitch_deg;
    input_attitude.yaw_deg = attitude.yaw_deg;
    input_attitude.thrust = attitude.thrust_value;
    input_attitude_topic.update(input_attitude);
    return;
}