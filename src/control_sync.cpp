#include "control_sync.hpp"

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