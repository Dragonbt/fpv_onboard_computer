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
    altitudeTest( telemetry, offboard );
    return;
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