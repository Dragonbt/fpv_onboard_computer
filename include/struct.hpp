#ifndef _STRUCT_
#define _STRUCT_

#include <chrono>

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

#define WRITE_VIDEO_WAIT_FOR_MS 50

#define PEER_SHUTDOWN -1
#define MAX_MSG_LENGTH 0x22FF
#define HEAD 0xAAAA
#define TAIL 0xDDDD

#define MAX_VEC_SIZE 40

#define HEART_BEAT_MSG 0
#define IMG_MSG 1
#define POSITION_MSG 2
#define VELOCITY_MSG 3
#define ATTITUDE_MSG 4
#define INPUT_MSG 5
#define STATUS_MSG 6
#define LOG_MSG 7

struct LogCommand{
    bool log = false;
    bool video = false;
};

struct MissionCommand{
    bool start = false;
};

struct ControlCommand{
    int index;
    double strength;
};

//sizeof PositionNED = 8*3 + 64/8 = 32bytes
typedef struct{
    double north_m = 0;
    double east_m = 0;
    double down_m = 0;
    int64_t time_ms = 0;
}PositionNED;

//sizeof VelocityNED = 8*3 + 64/8 = 32bytes
typedef struct{
    double north_m_s = 0;
    double east_m_s = 0;
    double down_m_s = 0;
    int64_t time_ms = 0;
}VelocityNED;

//sizeof EulerAngle = 8*3 + 64/8 = 32bytes
typedef struct{
    double roll_deg = 0;
    double pitch_deg = 0;
    double yaw_deg = 0;
    int64_t time_ms = 0;
}EulerAngle;

typedef struct{
    double forward_m_s = 0;
    double right_m_s = 0;
    double down_m_s = 0;
    double yawspeed_deg_s = 0;
    int64_t time_ms = 0;
}InputVelocityBody;

typedef struct{
    double roll_deg = 0;
    double pitch_deg = 0;
    double yaw_deg = 0;
    double thrust = 0;
    int64_t time_ms = 0;
}InputAttitude;

//sizeof Status = 8 + 8*3 + 56 = 88bytes
typedef struct{
    bool armed = false;
    bool in_air = false;
    bool rc_available_once = false;
    bool rc_available = false;
    double rc_signal_strength_percent = 0;
    double battery_voltage_v = 0;
    double battery_remaining_percent = 0;
    char flight_mode[50] = " ";
}Status;

#endif