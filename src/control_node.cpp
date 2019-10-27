#include "control_node.hpp"

void controlLoop( FileNode control_config )
{
    int enable;
    string url;
    float takeoff_altitude;

    float Kp_z, Ki_z, Kd_z;

    control_config["ENABLE"] >> enable;
    control_config["URL"] >> url;
    control_config["TAKEOFF_ALTITUDE"] >> takeoff_altitude;
    control_config["Kp_z"] >> Kp_z;
    control_config["Ki_z"] >> Ki_z;
    control_config["Kd_z"] >> Kd_z;
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
    altitudeTest( telemetry, offboard, Kp_z, Ki_z, Kd_z );
    cout << "[WARNING]: control node shut down" << endl;
    return;
}