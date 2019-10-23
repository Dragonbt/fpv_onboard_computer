#include "control_node.hpp"

void controlLoop( FileNode control_config )
{
    int enable;
    string url;
    float takeoff_altitude;

    control_config["ENABLE"] >> enable;
    control_config["URL"] >> url;
    control_config["TAKEOFF_ALTITUDE"] >> takeoff_altitude;
    if( enable == 0 )
    {
        cout << "[WARNING]: control node disabled" << endl;
        return;
    }

    /*Connect to pixhawk*/
    Mavsdk dc;
    ConnectionResult connection_result;
    connection_result = dc.add_any_connection(url);
    if (connection_result != ConnectionResult::SUCCESS) {
        cout <<  string("[ERROR]: ") + connection_result_str(connection_result) << endl;
        cout << "[WARNING]: control node shut down" << endl;
        return;
    }
    while (!dc.is_connected()) {
        cout << "[LOGGING]: Wait for system to connect via heartbeat" << endl;
        this_thread::sleep_for(seconds(1));
    }
    System& system = dc.system();
    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);
    /*Health Check*/
    //healthCheck( telemetry );
    setTelemetry( telemetry );
    /*Arm*/
    //arm( telemetry, action );
    /*Takeoff*/
    //takeoff( telemetry, action, takeoff_altitude );
    //this_thread::sleep_for(seconds(10));
    //waitForArmed( telemetry );
    //attitudeTest(telemetry, action, offboard);
    //cout << "[LOGGING]: land success" << endl;
    //cout << "[WARNING]: control node shut down" << endl;
    altitudeTest( telemetry, offboard );
    return;
}

void setTelemetry( shared_ptr<Telemetry> telemetry )
{
    Telemetry::Result set_rate_result;
    set_rate_result = telemetry->set_rate_position_velocity_ned( 250 );
    if (set_rate_result != Telemetry::Result::SUCCESS) {
        cout << string("[ERROR]: ") + Telemetry::result_str(set_rate_result) << endl;
        return;
    }
    set_rate_result = telemetry->set_rate_attitude( 250 );
    if (set_rate_result != Telemetry::Result::SUCCESS) {
        cout << string("[ERROR]: ") + Telemetry::result_str(set_rate_result) << endl;
        return;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry->position_velocity_ned_async([](Telemetry::PositionVelocityNED position_velocity_ned){
        PositionNED position;
        VelocityNED velocity;

        position.north_m = position_velocity_ned.position.north_m;
        position.east_m = position_velocity_ned.position.east_m;
        position.down_m = position_velocity_ned.position.down_m;
        position.time_ms = intervalMs( high_resolution_clock::now(), init_timepoint );

        velocity.north_m_s = position_velocity_ned.velocity.north_m_s;
        velocity.east_m_s = position_velocity_ned.velocity.east_m_s;
        velocity.down_m_s = position_velocity_ned.velocity.down_m_s;
        velocity.time_ms = intervalMs( high_resolution_clock::now(), init_timepoint );
        
        position_vec_mutex.lock();
        if( position_vec_topic.size() > MAX_VEC_SIZE )
        {
            position_vec_topic.clear();
        }
        position_vec_topic.push_back(position);
        position_vec_mutex.unlock();

        position_vec_log_mutex.lock();
        if( position_vec_log_topic.size() > MAX_VEC_SIZE )
        {
            position_vec_log_topic.clear();
        }
        position_vec_log_topic.push_back(position);
        position_vec_log_mutex.unlock();

        velocity_vec_mutex.lock();
        if( velocity_vec_topic.size() > MAX_VEC_SIZE )
        {
            velocity_vec_topic.clear();
        }
        velocity_vec_topic.push_back(velocity);
        velocity_vec_mutex.unlock();

        velocity_vec_log_mutex.lock();
        if( velocity_vec_log_topic.size() > MAX_VEC_SIZE )
        {
            velocity_vec_log_topic.clear();
        }
        velocity_vec_log_topic.push_back(velocity);
        velocity_vec_log_mutex.unlock();
        //cout << string("[LOGGING]: altitude speed ") << position_velocity_ned.velocity.down_m_s << endl;
    });

    telemetry->attitude_euler_angle_async([](Telemetry::EulerAngle attitude_euler_angle){
        EulerAngle euler_angle;
        euler_angle.roll_deg = attitude_euler_angle.roll_deg;
        euler_angle.pitch_deg = attitude_euler_angle.pitch_deg;
        euler_angle.yaw_deg = attitude_euler_angle.yaw_deg;
        euler_angle.time_ms = intervalMs( high_resolution_clock::now(), init_timepoint );
        
        euler_angle_vec_mutex.lock();
        if( euler_angle_vec_topic.size() > MAX_VEC_SIZE )
        {
            euler_angle_vec_topic.clear();
        }
        euler_angle_vec_topic.push_back(euler_angle);
        euler_angle_vec_mutex.unlock();

        euler_angle_vec_log_mutex.lock();
        if( euler_angle_vec_log_topic.size() > MAX_VEC_SIZE )
        {
            euler_angle_vec_log_topic.clear();
        }
        euler_angle_vec_log_topic.push_back(euler_angle);
        euler_angle_vec_log_mutex.unlock();
        //cout << string("[LOGGING]: yaw ") << euler_angle.yaw_deg << endl;
    });

    telemetry->armed_async([](bool armed){
        status_mutex.lock();
        status.armed = armed;
        if( status_topic.size() > MAX_VEC_SIZE )
        {
            status_topic.clear();
        }
        status_topic.push_back(status);
        status_mutex.unlock();
    });

    telemetry->in_air_async([](bool in_air){
        status_mutex.lock();
        status.in_air = in_air;
        if( status_topic.size() > MAX_VEC_SIZE )
        {
            status_topic.clear();
        }
        status_topic.push_back(status);
        status_mutex.unlock();
    });

    telemetry->rc_status_async([](Telemetry::RCStatus rc_status){
        status_mutex.lock();
        status.rc_available_once = rc_status.available_once;
        status.rc_available = rc_status.available;
        status.rc_signal_strength_percent = rc_status.signal_strength_percent;
        if( status_topic.size() > MAX_VEC_SIZE )
        {
            status_topic.clear();
        }
        status_topic.push_back(status);
        status_mutex.unlock();
    });

    telemetry->battery_async([](Telemetry::Battery battery){
        status_mutex.lock();
        status.battery_voltage_v = battery.voltage_v;
        status.battery_remaining_percent = battery.remaining_percent;
        if( status_topic.size() > MAX_VEC_SIZE )
        {
            status_topic.clear();
        }
        status_topic.push_back(status);
        status_mutex.unlock();
    });

    telemetry->flight_mode_async([](Telemetry::FlightMode flight_mode){
        string mode = Telemetry::flight_mode_str(flight_mode);
        status_mutex.lock();
        strncpy(status.flight_mode, mode.c_str(), sizeof(status.flight_mode));
        status.flight_mode[sizeof(status.flight_mode) - 1] = 0;
        if( status_topic.size() > MAX_VEC_SIZE )
        {
            status_topic.clear();
        }
        status_topic.push_back(status);
        status_mutex.unlock();
    });

    telemetry->status_text_async([](Telemetry::StatusText status_text){
        string prefix;
        switch (status_text.type){
            case Telemetry::StatusText::StatusType::CRITICAL:
                prefix = "[CRITICAL]: ";
                break;
            case Telemetry::StatusText::StatusType::INFO:
                prefix = "[INFO]: ";
                break;
            case Telemetry::StatusText::StatusType::WARNING:
                prefix = "[WARNING]: ";
                break;
            default:
                prefix = "[UNKNOWN]: ";
                break;
        }
        string msg = prefix + status_text.text;
        string_vec_mutex.lock();
        string_vec_topic.push_back(msg);
        string_vec_mutex.unlock();
    });
    return;
}

void healthCheck( shared_ptr<Telemetry> telemetry )
{
    while ( ! telemetry->health_all_ok() ) {
        cout << "[LOGGING]: Waiting for health_all_ok" << endl;
        this_thread::sleep_for(seconds(1));
    }
}

/*
void arm( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action )
{
    GCCommand command;
    Action::Result arm_result;
    while( ! telemetry->armed() )
    {
        command_mutex.lock();
        command = command_topic;
        command_mutex.unlock();
        if( command.arm )
        {
            arm_result = action->arm();
            if ( arm_result != Action::Result::SUCCESS ){
                cout << string("[ERROR]: ") + Action::result_str(arm_result) << endl;
                cout << "[ERROR]: unable to arm" << endl;
            }
        }
        this_thread::sleep_for( seconds(1) );
    }
    cout << "[LOGGING]: arm success, ready to takeoff" << endl;
}
*/
/*
void takeoff( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action, float altitude )
{
    GCCommand command;
    Action::Result set_takeoff_altitude_result, takeoff_result;
    while( ! telemetry->in_air() )
    {
        command_mutex.lock();
        command = command_topic;
        command_mutex.unlock();
        if( command.takeoff )
        {
            set_takeoff_altitude_result = action->set_takeoff_altitude( altitude );
            if ( set_takeoff_altitude_result != Action::Result::SUCCESS ){
                cout << string("[ERROR]: ") + Action::result_str(set_takeoff_altitude_result) << endl;
            }
            takeoff_result = action->takeoff();
            if ( takeoff_result != Action::Result::SUCCESS ){
                cout << string("[ERROR]: ") + Action::result_str(takeoff_result) << endl;
            }
        }
        this_thread::sleep_for( seconds(1) );
    }
}
*/
/*
void land( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action )
{
    Action::Result land_result;
    while( telemetry->in_air() )
    {
        land_result = action->land();
        if ( land_result != Action::Result::SUCCESS ){
            cout << string("[ERROR]: ") + Action::result_str(land_result) << endl;
        }
        this_thread::sleep_for( seconds(1) );
    }
}
*/
/*
bool readControlCommand( GCCommand &command )
{
    GCCommand empty_command;
    command_mutex.lock();
    command = command_topic;
    command_topic = empty_command;
    command_mutex.unlock();
    if( command == empty_command ){
        return false;
    }
    else{
        return true;
    }
}
*/

void waitForArmed( shared_ptr<Telemetry> telemetry )
{
    while( ! telemetry->armed() )
    {
        this_thread::sleep_for(seconds(1));
    }
    cout << "[LOGGING]: armed" << endl;
    return;
}

void pushInputVelocityBody( Offboard::VelocityBodyYawspeed velocity )
{
    InputVelocityBody input;
    input.forward_m_s = velocity.forward_m_s;
    input.right_m_s = velocity.right_m_s;
    input.down_m_s = velocity.down_m_s;
    input.yawspeed_deg_s = velocity.yawspeed_deg_s;
    input.time_ms = intervalMs(high_resolution_clock::now(), init_timepoint);
    input_velocity_body_vec_mutex.lock();
    if( input_velocity_body_vec_topic.size() > MAX_VEC_SIZE )
    {
        input_velocity_body_vec_topic.clear();
    }
    input_velocity_body_vec_topic.push_back( input );
    input_velocity_body_vec_mutex.unlock();

    input_velocity_body_vec_log_mutex.lock();
    if( input_velocity_body_vec_log_topic.size() > MAX_VEC_SIZE )
    {
        input_velocity_body_vec_log_topic.clear();
    }
    input_velocity_body_vec_log_topic.push_back( input );
    input_velocity_body_vec_log_mutex.unlock();
}
void pushInputAttitude( Offboard::Attitude attitude )
{
    InputAttitude input;
    input.roll_deg = attitude.roll_deg;
    input.pitch_deg = attitude.pitch_deg;
    input.yaw_deg = attitude.yaw_deg;
    input.thrust = attitude.thrust_value;
    input.time_ms = intervalMs(high_resolution_clock::now(), init_timepoint);
    input_attitude_vec_mutex.lock();
    if( input_attitude_vec_topic.size() > MAX_VEC_SIZE )
    {
        input_attitude_vec_topic.clear();
    }
    input_attitude_vec_topic.push_back( input );
    input_attitude_vec_mutex.unlock();

    input_attitude_vec_log_mutex.lock();
    if( input_attitude_vec_log_topic.size() > MAX_VEC_SIZE )
    {
        input_attitude_vec_log_topic.clear();
    }
    input_attitude_vec_log_topic.push_back( input );
    input_attitude_vec_log_mutex.unlock();
}

void offbCtrlAttitude(shared_ptr<Offboard> offboard, Offboard::Attitude attitude)
{
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
    return;
}

void offbCtrlVelocityBody( std::shared_ptr<mavsdk::Offboard> offboard, Offboard::VelocityBodyYawspeed velocity )
{
    Offboard::Result offboard_result;
    if( ! offboard->is_active() ){
        offboard->set_velocity_body(velocity);
        offboard_result = offboard->start();
        if ( offboard_result != Offboard::Result::SUCCESS ){
            cout << string("[ERROR]: ") + Offboard::result_str(offboard_result) << endl;
            cout << "[ERROR]: unable to start offboard" << endl;
            return;
        }
    }
    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_velocity_body(velocity);
    return;
}

void offbCtrlPositionNED( std::shared_ptr<mavsdk::Offboard> offboard, Offboard::PositionNEDYaw position )
{
    Offboard::Result offboard_result;
    if( ! offboard->is_active() ){
        offboard->set_position_ned( position );
        offboard_result = offboard->start();
        if ( offboard_result != Offboard::Result::SUCCESS ){
            cout << string("[ERROR]: ") + Offboard::result_str(offboard_result) << endl;
            cout << "[ERROR]: unable to start offboard" << endl;
            return;
        }
    }
    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_position_ned( position );
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