#include "telem_async.hpp"

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
        PositionNED position_ned;
        PositionBody position_body;
        VelocityNED velocity_ned;
        VelocityBody velocity_body;
        EulerAngle attitude;
        float yaw_rad;
        int64_t timestamp;
        
        position_ned.north_m = position_velocity_ned.position.north_m;
        position_ned.east_m = position_velocity_ned.position.east_m;
        position_ned.down_m = position_velocity_ned.position.down_m;
        position_ned_topic.update(position_ned);

        velocity_ned.north_m_s = position_velocity_ned.velocity.north_m_s;
        velocity_ned.east_m_s = position_velocity_ned.velocity.east_m_s;
        velocity_ned.down_m_s = position_velocity_ned.velocity.down_m_s;
        velocity_ned_topic.update(velocity_ned);

        if( attitude_topic.latest(timestamp, attitude) )
        {
            yaw_rad = attitude.yaw_deg * 3.14 / 180;
            position_body.x_m = position_ned.north_m * cos(yaw_rad) + position_ned.east_m * sin(yaw_rad);
            position_body.y_m = position_ned.north_m * sin(-yaw_rad) + position_ned.east_m * cos(yaw_rad);
            position_body.z_m = position_ned.down_m;
            position_body_topic.update(position_body);

            velocity_body.x_m_s = velocity_ned.north_m_s * cos(yaw_rad) + velocity_ned.east_m_s * sin(yaw_rad);
            velocity_body.y_m_s = velocity_ned.north_m_s * sin(-yaw_rad) + velocity_ned.east_m_s * cos(yaw_rad);
            velocity_body.z_m_s = velocity_ned.down_m_s;
            velocity_body_topic.update(velocity_body);
        }
    });

    telemetry->attitude_euler_angle_async([](Telemetry::EulerAngle attitude_euler_angle){
        EulerAngle euler_angle;
        euler_angle.roll_deg = attitude_euler_angle.roll_deg;
        euler_angle.pitch_deg = attitude_euler_angle.pitch_deg;
        euler_angle.yaw_deg = attitude_euler_angle.yaw_deg;
        attitude_topic.update(euler_angle);
    });

    telemetry->armed_async([](bool armed){
        VehicleStatus vehicle_status;
        int64_t timestamp;
        vehicle_status_topic.latest(timestamp, vehicle_status);
        vehicle_status.armed = armed;
        vehicle_status_topic.update(vehicle_status);
    });

    telemetry->in_air_async([](bool in_air){
        VehicleStatus vehicle_status;
        int64_t timestamp;
        vehicle_status_topic.latest(timestamp, vehicle_status);
        vehicle_status.in_air = in_air;
        vehicle_status_topic.update(vehicle_status);
    });

    telemetry->rc_status_async([](Telemetry::RCStatus rc_status){
        VehicleStatus vehicle_status;
        int64_t timestamp;
        vehicle_status_topic.latest(timestamp, vehicle_status);
        vehicle_status.rc_available_once = rc_status.available_once;
        vehicle_status.rc_available = rc_status.available;
        vehicle_status.rc_signal_strength_percent = rc_status.signal_strength_percent;
        vehicle_status_topic.update(vehicle_status);
    });

    telemetry->battery_async([](Telemetry::Battery battery){
        VehicleStatus vehicle_status;
        int64_t timestamp;
        vehicle_status_topic.latest(timestamp, vehicle_status);
        vehicle_status.battery_voltage_v = battery.voltage_v;
        vehicle_status.battery_remaining_percent = battery.remaining_percent;
        vehicle_status_topic.update(vehicle_status);
    });

    telemetry->flight_mode_async([](Telemetry::FlightMode flight_mode){
        string mode = Telemetry::flight_mode_str(flight_mode);
        VehicleStatus vehicle_status;
        int64_t timestamp;
        vehicle_status_topic.latest(timestamp, vehicle_status);
        strncpy(vehicle_status.flight_mode, mode.c_str(), sizeof(vehicle_status.flight_mode));
        vehicle_status.flight_mode[sizeof(vehicle_status.flight_mode) - 1] = 0;
        vehicle_status_topic.update(vehicle_status);
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
        string_topic.update(msg);
    });
    return;
}