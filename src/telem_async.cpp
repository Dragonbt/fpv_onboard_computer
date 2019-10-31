#include "telem_async.hpp"

Status status;

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
        /*
        position.north_m = position_velocity_ned.position.north_m * sin(yaw) + position_velocity_ned.position.east_m * cos(yaw);
		position.east_m = position_velocity_ned.position.north_m * cos(yaw) - position_velocity_ned.position.east_m * sin(yaw);
        */
        position.time_ms = intervalMs( high_resolution_clock::now(), init_timepoint );

        velocity.north_m_s = position_velocity_ned.velocity.north_m_s;
        velocity.east_m_s = position_velocity_ned.velocity.east_m_s;
        velocity.down_m_s = position_velocity_ned.velocity.down_m_s;
        velocity.time_ms = intervalMs( high_resolution_clock::now(), init_timepoint );
        
        position_mutex.lock();
        while( position_topic.size() >= MAX_VEC_SIZE )
        {
            position_topic.pop_front();
        }
        position_topic.push_back(position);
        position_mutex.unlock();

        velocity_mutex.lock();
        while( velocity_topic.size() >= MAX_VEC_SIZE )
        {
            velocity_topic.pop_front();
        }
        velocity_topic.push_back(velocity);
        velocity_mutex.unlock();

        writePositionNED(position);
        writeVelocityNED(velocity);
    });

    telemetry->attitude_euler_angle_async([](Telemetry::EulerAngle attitude_euler_angle){
        EulerAngle euler_angle;
        euler_angle.roll_deg = attitude_euler_angle.roll_deg;
        euler_angle.pitch_deg = attitude_euler_angle.pitch_deg;
        euler_angle.yaw_deg = attitude_euler_angle.yaw_deg;
        euler_angle.time_ms = intervalMs( high_resolution_clock::now(), init_timepoint );
        
        attitude_mutex.lock();
        while( attitude_topic.size() >= MAX_VEC_SIZE )
        {
            attitude_topic.pop_front();
        }
        attitude_topic.push_back(euler_angle);
        attitude_mutex.unlock();
        writeAttitude(euler_angle);
    });

    telemetry->armed_async([](bool armed){
        status_mutex.lock();
        status.armed = armed;
        if( status_topic.size() >= MAX_VEC_SIZE )
        {
            status_topic.clear();
        }
        status_topic.push_back(status);
        status_mutex.unlock();
    });

    telemetry->in_air_async([](bool in_air){
        status_mutex.lock();
        status.in_air = in_air;
        if( status_topic.size() >= MAX_VEC_SIZE )
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
        if( status_topic.size() >= MAX_VEC_SIZE )
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
        if( status_topic.size() >= MAX_VEC_SIZE )
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
        if( status_topic.size() >= MAX_VEC_SIZE )
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
        string_mutex.lock();
        if( string_topic.size() >= MAX_VEC_SIZE )
        {
            string_topic.clear();
        }
        string_topic.push_back(msg);
        string_mutex.unlock();
    });
    return;
}