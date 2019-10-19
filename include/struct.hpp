#ifndef _STRUCT_
#define _STRUCT_

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

#define WRITE_VIDEO_WAIT_FOR_MS 50

#define PEER_SHUTDOWN -1
#define MAX_MSG_LENGTH 0x22FF
#define HEAD 0xAAAA
#define TAIL 0xDDDD

#define MAX_VEC_SIZE 40

#include <chrono>

typedef struct{
    bool arm = false;
    bool takeoff = false;
    bool land = false;
    bool up = false;
    bool down = false;
    bool forward = false;
    bool backward = false;
    bool left = false;
    bool right = false;
    bool log = false;
    bool video = false;
}GCCommand;

typedef struct{
    double north_m = 0;
    double east_m = 0;
    double down_m = 0;
    int64_t time_ms = 0;
}PositionNED;

typedef struct{
    double north_m_s = 0;
    double east_m_s = 0;
    double down_m_s = 0;
    int64_t time_ms = 0;
}VelocityNED;

typedef struct{
    double roll_deg = 0;
    double pitch_deg = 0;
    double yaw_deg = 0;
    int64_t time_ms = 0;
}EulerAngle;

typedef struct{
    float forward_m_s;
    float right_m_s;
    float down_m_s;
    float yawspeed_deg_s;
    int64_t time_ms = 0;
}Input;


#endif