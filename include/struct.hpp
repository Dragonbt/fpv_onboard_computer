#ifndef _STRUCT_
#define _STRUCT_

#include <chrono>

struct MissionCommand{
    int16_t index = -1;
    double strength = 0;
};

//sizeof PositionNED = 8*3 + 64/8 = 32bytes
struct PositionNED{
    double north_m = 0;
    double east_m = 0;
    double down_m = 0;
    int64_t time_ms = 0;
};

//sizeof VelocityNED = 8*3 + 64/8 = 32bytes
struct VelocityNED{
    double north_m_s = 0;
    double east_m_s = 0;
    double down_m_s = 0;
    int64_t time_ms = 0;
};

//sizeof EulerAngle = 8*3 + 64/8 = 32bytes
struct EulerAngle{
    double roll_deg = 0;
    double pitch_deg = 0;
    double yaw_deg = 0;
    int64_t time_ms = 0;
};

struct InputVelocityBody{
    double forward_m_s = 0;
    double right_m_s = 0;
    double down_m_s = 0;
    double yawspeed_deg_s = 0;
    int64_t time_ms = 0;
};

//sizeof InputAttitude = 8*4 + 8 = 40
struct InputAttitude{
    double roll_deg = 0;
    double pitch_deg = 0;
    double yaw_deg = 0;
    double thrust = 0;
    int64_t time_ms = 0;
};

//sizeof Status = 8 + 8*3 + 56 = 88bytes
struct Status{
    bool armed = false;
    bool in_air = false;
    bool rc_available_once = false;
    bool rc_available = false;
    double rc_signal_strength_percent = 0;
    double battery_voltage_v = 0;
    double battery_remaining_percent = 0;
    char flight_mode[50] = " ";
};

struct Reference{
    double down_m;
    int64_t time_ms;
};

#endif