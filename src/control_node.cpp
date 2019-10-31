#include "control_node.hpp"

void controlLoop( FileNode control_config )
{
    int enable;
    string url;
    float takeoff_altitude;

    float Kp_z, Ki_z, Kd_z, Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y;

    control_config["ENABLE"] >> enable;
    control_config["URL"] >> url;
    control_config["TAKEOFF_ALTITUDE"] >> takeoff_altitude;
    control_config["Kp_z"] >> Kp_z;
    control_config["Ki_z"] >> Ki_z;
    control_config["Kd_z"] >> Kd_z;

    control_config["Kp_x"] >> Kp_x;
    control_config["Ki_x"] >> Ki_x;
    control_config["Kd_x"] >> Kd_x;

    control_config["Kp_y"] >> Kp_y;
    control_config["Ki_y"] >> Ki_y;
    control_config["Kd_y"] >> Kd_y;
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
    altitudeTest( telemetry, offboard, Kp_z, Ki_z, Kd_z, Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y );
    cout << "[WARNING]: control node shut down" << endl;
    return;
}