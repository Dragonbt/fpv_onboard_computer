#include "control_node.hpp"
#include "pidcontroller.hpp"

double pre_vel_err_z=0.0, _pos_sp_z,_vel_sp_z=0.0, _thr_int_z=0.0;
int SampleTime = 20;
float yaw;
Telemetry::PositionVelocityNED position;

void altitudeTest( shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard )
{
    MissionCommand command;
    Telemetry::EulerAngle euler_angle;
    double time_change = 0;
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    while(true)
    {
        mission_command_mutex.lock();
        command = mission_command_topic;
        mission_command_mutex.unlock();
        if( command.start )
        {
            position = telemetry->position_velocity_ned();
            _pos_sp_z = position.position.down_m;
            euler_angle = telemetry->attitude_euler_angle();
            yaw = euler_angle.yaw_deg;
            remotePrint(string("start mission"));
            remotePrint(string("enter offboard !"));
            break;
        }
        this_thread::sleep_for(milliseconds(100));
    }
    while(true)
    {
        mission_command_mutex.lock();
        command = mission_command_topic;
        mission_command_mutex.unlock();
        if( ! command.start ){
            quitOffboard( offboard );
            remotePrint(string("quit mission"));
            break;
        }
        time_change = intervalMs( high_resolution_clock::now(), t0);
        if( time_change < SampleTime ){
            continue;
        }
        t0 = high_resolution_clock::now();
        //control code
        altitude(telemetry, offboard, time_change);
    }
    return;
}

void altitude(shared_ptr<Telemetry> telemetry, shared_ptr<Offboard> offboard, double dt)
{
    Offboard::Attitude attitude;
	double _pos_z, _vel_z, thrust, vel_sp_pos_z;
	double Kp_z = 1, Ki_z = 0, Kd_z = 0;
	double Kp_Vz = 0.2, Ki_Vz = 0.02, Kd_Vz = 0;
    position = telemetry->position_velocity_ned();
	_pos_z = position.position.down_m;//¶ÁÏÖÔÚžß¶È
	PID myPID_z(&_pos_z, &vel_sp_pos_z, &_pos_sp_z, Kp_z, Ki_z, Kd_z, DIRECT);
	myPID_z.SetSampleTime(dt);//ÔÝ¶šÖÜÆÚ20ms
	myPID_z.SetOutputLimits(-10, 10);
	myPID_z.SetMode(AUTOMATIC);
	myPID_z.Compute();
	_vel_sp_z += vel_sp_pos_z;
	_vel_sp_z = _vel_sp_z > -3 ? (_vel_sp_z < 1 ? _vel_sp_z : 1) : -3;
	_vel_z = position.velocity.down_m_s;//¶Ážß¶È·œÏòµÄËÙ¶ÈÐÅÏ¢
	const double vel_err_z = _vel_sp_z - _vel_z;
	const double acc_err_z = (vel_err_z - pre_vel_err_z) / dt;//Ä¿²âÔëÉùŽóµŒÖÂÎ¢·ÖÏîÃ»·šÓÃ
															  // Consider thrust in D-direction.
    pre_vel_err_z = vel_err_z;
	float thrust_desired_D = Kp_Vz * vel_err_z + Kd_Vz * acc_err_z + _thr_int_z
		- 0.5;

	// The Thrust limits are negated and swapped due to NED-frame.
	float uMax = -0.06f;
	float uMin = -1.0f;

	// make sure there's always enough thrust vector length to infer the attitude
	uMax = uMax < -10e-4f ? uMax : -10e-4f;

	// Apply Anti-Windup in D-direction.
	bool stop_integral_D = (thrust_desired_D >= uMax && vel_err_z >= 0.0f) ||
		(thrust_desired_D <= uMin && vel_err_z <= 0.0f);

	if (!stop_integral_D) {
		_thr_int_z += vel_err_z * Ki_Vz * dt;

		// limit thrust integral
		int sign_thr_int_z = _thr_int_z > 0 ? 1 : -1;

		_thr_int_z = std::min(fabsf(_thr_int_z), 1.0f) * sign_thr_int_z;
	}

	// Saturate thrust setpoint in D-direction.
	thrust = thrust_desired_D > uMin ? (thrust_desired_D < uMax ? thrust_desired_D : uMax) : uMin;
	thrust = -thrust;
	cout << "pos_sp_z: " << _pos_sp_z << " pos_z: " << _pos_z << " vel_sp_pos_z: " << vel_sp_pos_z << endl;
	cout << "vel_sp_z: " << _vel_sp_z << " vel_z: " << _vel_z << " thrust: " << thrust << endl;
    
    attitude = {0.0f, 0.0f, yaw, (float)thrust};
    offbCtrlAttitude(offboard, attitude);
    pushInputAttitude(attitude);
	return;
}

/*
void attitudeTest( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action, shared_ptr<Offboard> offboard )
{
    GCCommand command;
    //Offboard::VelocityBodyYawspeed velocity;
    Offboard::Attitude attitude;
    //Offboard::PositionNEDYaw position;
    //InputVelocityBody input_velocity;
    //InputAttitude input_attitude;
    remotePrint(string("Enter test loop"));
    while( true )
    {
        if( ! readControlCommand( command ) ){
            //cout << "no command arrive" << endl;
            //attitude = {0.0f, 0.0f, 0.0f, 0.5f};
            //offbCtrlAttitude(offboard, attitude);
            //pushInput( velocity, attitude );
            this_thread::sleep_for(milliseconds(30));
            continue;
        }
        if( command.quit )
        {
            quitOffboard( offboard );
            continue;
        }
        if( command.land )
        {
            cout << "LAND" << endl;
            land( telemetry, action );
            break;
        }
        if( command.up )
        {
            cout << "UP" << endl;
            //position = {0.0f, 0.0f, -10.0f, 0.0f};
            //offbCtrlPositionNED( offboard, position );
            attitude = {0.0f, 0.0f, 0.0f, 0.7f};
            offbCtrlAttitude(offboard, attitude);
            //pushInputAttitude(attitude);
        }
        if( command.down )
        {
            //cout << "DOWN" << endl;
            //position = {0.0f, 0.0f, -10.0f, 0.0f};
            //offbCtrlPositionNED( offboard, position );
            attitude = {0.0f, 0.0f, 0.0f, 0.3f};
            offbCtrlAttitude(offboard, attitude);
            //pushInputAttitude(attitude);
        }
        if( command.forward )
        {
            cout << "FORWARD" << endl;
            //position = {10.0f, 0.0f, 0.0f, 0.0f};
            //offbCtrlPositionNED( offboard, position );
            attitude = {0.0f, -10.0f, 0.0f, 0.5f};
            offbCtrlAttitude(offboard, attitude);
            //pushInputAttitude(attitude);
        }
        if( command.backward )
        {
            cout << "BACKWARD" << endl;
            //position = {-10.0f, 0.0f, 0.0f, 0.0f};
            //offbCtrlPositionNED( offboard, position );
            attitude = {0.0f, 10.0f, 0.0f, 0.5f};
            offbCtrlAttitude(offboard, attitude);
            //pushInputAttitude(attitude);
        }
        if( command.left )
        {
            cout << "LEFT" << endl;
            //velocity = {0.0f, -10.0f, 0.0f, 0.0f};
            //offbCtrlVelocityBody(offboard, velocity);
            attitude = {-10.0f, 0.0f, 0.0f, 0.5f};
            offbCtrlAttitude(offboard, attitude);
            //pushInputAttitude(attitude);
        }
        if( command.right )
        {
            cout << "RIGHT" << endl;
            //velocity = {0.0f, 10.0f, 0.0f, 0.0f};
            //offbCtrlVelocityBody(offboard, velocity);
            attitude = {10.0f, 0.0f, 0.0f, 0.5f};
            offbCtrlAttitude(offboard, attitude);
            //pushInputAttitude(attitude);
        }
        if( command.yaw_pos )
        {
            cout << "YAW+" << endl;
            //position = {0.0f, 0.0f, 0.0f, 10.0f};
            //offbCtrlPositionNED(offboard, position);
            //velocity = {0.0f, 0.0f, 0.0f, 10.0f};
            //offbCtrlVelocityBody(offboard, velocity);
            //pushInputVelocityBody(velocity);
            attitude = {0.0f, 0.0f, 10.0f, 0.5f};
            offbCtrlAttitude(offboard, attitude);
        }
        if( command.yaw_neg )
        {
            cout << "YAW-" << endl;
            //position = {0.0f, 0.0f, 0.0f, -10.0f};
            //offbCtrlPositionNED(offboard, position);
            //velocity = {0.0f, 0.0f, 0.0f, -10.0f};
            //offbCtrlVelocityBody(offboard, velocity);
            //pushInputVelocityBody(velocity);
            attitude = {0.0f, 0.0f, -10.0f, 0.5f};
            offbCtrlAttitude(offboard, attitude);
        }
        this_thread::sleep_for(milliseconds(30));    
    }
    return;
}
*/