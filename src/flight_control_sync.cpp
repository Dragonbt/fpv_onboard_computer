#include "flight_control_sync.hpp"

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
    pushInputAttitude(attitude);
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
    pushInputVelocityBody(velocity);
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