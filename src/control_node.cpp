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
    waitForOffboard( offboard );
    test(telemetry, action, offboard);
    cout << "[LOGGING]: land success" << endl;
    cout << "[WARNING]: control node shut down" << endl;
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
    return;
}

void healthCheck( shared_ptr<Telemetry> telemetry )
{
    while ( ! telemetry->health_all_ok() ) {
        cout << "[LOGGING]: Waiting for health_all_ok" << endl;
        this_thread::sleep_for(seconds(1));
    }
}

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

void clearOffboardCommand()
{
    command_mutex.lock();
    command_topic.up = false;
    command_topic.down = false;
    command_topic.left = false;
    command_topic.right = false;
    command_topic.forward = false;
    command_topic.backward = false;
    command_topic.yaw_pos = false;
    command_topic.yaw_neg = false;
    command_topic.thrust = false;
    command_mutex.unlock();
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

void waitForOffboard( shared_ptr<Offboard> offboard ){
    Offboard::Result offboard_result;
    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
    while( true )
    {
        offboard_result = offboard->start();
        if ( offboard_result != Offboard::Result::SUCCESS ){
            cout << string("[ERROR]: ") + Offboard::result_str(offboard_result) << endl;
            cout << "[ERROR]: unable to start offboard" << endl;
        }
        else{
            break;
        }
        this_thread::sleep_for(seconds(1));
    }
    cout << "[LOGGING]: offboard mode start" << endl;
    return;
}

void pushInput( Input input )
{
    input_vec_mutex.lock();
    if( input_vec_topic.size() > MAX_VEC_SIZE )
    {
        input_vec_topic.clear();
    }
    input_vec_topic.push_back( input );
    input_vec_mutex.unlock();

    input_vec_log_mutex.lock();
    if( input_vec_log_topic.size() > MAX_VEC_SIZE )
    {
        input_vec_log_topic.clear();
    }
    input_vec_log_topic.push_back( input );
    input_vec_log_mutex.unlock();
}
void test( shared_ptr<Telemetry> telemetry, shared_ptr<Action> action, shared_ptr<Offboard> offboard )
{
    GCCommand command;
    Offboard::VelocityBodyYawspeed u;
    Input input;
    while( true )
    {
        command_mutex.lock();
        command = command_topic;
        command_mutex.unlock();
        if( command.land )
        {
            land( telemetry, action );
            break;
        }
        else{
            clearOffboardCommand();
            u = {0.0f, 0.0f, 0.0f, 0.0f};
            offboard->set_velocity_body( {0.0f, 0.0f, 0.0f, 0.0f} );
            cout << "set to 0" << endl;
            if( command.up )
            {
                cout << "UP" << endl;
                u = {0.0f, 0.0f, -0.5f, 0.0f};
                offboard->set_velocity_body( u );
            }
            if( command.down )
            {
                cout << "DOWN" << endl;
                u = {0.0f, 0.0f, 0.5f, 0.0f};
                offboard->set_velocity_body( u );
            }
            if( command.forward )
            {
                cout << "FORWARD" << endl;
                u = {0.5f, 0.0f, 0.0f, 0.0f};
                offboard->set_velocity_body( u );
            }
            if( command.backward )
            {
                cout << "BACKWARD" << endl;
                u = {-0.5f, 0.0f, 0.0f, 0.0f};
                offboard->set_velocity_body( u );
            }
            if( command.left )
            {
                cout << "LEFT" << endl;
                u = {0.0f, 0.0f, -0.5f, 0.0f};
                offboard->set_velocity_body( u );
            }
            if( command.right )
            {
                cout << "RIGHT" << endl;
                u = {0.0f, 0.0f, 0.5f, 0.0f};
                offboard->set_velocity_body( u );
            }
            if( command.yaw_pos )
            {
                cout << "YAW+" << endl;
                u = {0.0f, 0.0f, 0.0f, 10.0f};
                offboard->set_velocity_body( u );
            }
            if( command.yaw_neg )
            {
                cout << "YAW-" << endl;
                u = {0.0f, 0.0f, 0.0f, -10.0f};
                offboard->set_velocity_body( u );
            }
            if( command.thrust )
            {
                cout << "thrust" << endl;
                Offboard::Attitude att = {0.0f, 0.0f, 0.0f, 0.4f};
                offboard->set_attitude( att );
            }
            input.forward_m_s = u.forward_m_s;
            input.right_m_s = u.right_m_s;
            input.down_m_s = u.down_m_s;
            input.yawspeed_deg_s = u.yawspeed_deg_s;
            input.time_ms = intervalMs(high_resolution_clock::now(), init_timepoint);
            pushInput( input );
            this_thread::sleep_for(milliseconds(300));
        }
    }
    return;
}