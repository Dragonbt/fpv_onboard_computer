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
    bool yaw_pos = false;
    bool yaw_neg = false;
    bool log = false;
    bool video = false;
}GCCommand;

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
    float forward_m_s = 0;
    float right_m_s = 0;
    float down_m_s = 0;
    float yawspeed_deg_s = 0;
    float roll_deg = 0;
    float pitch_deg = 0;
    float yaw_deg = 0;
    float thrust = 0;
    int64_t time_ms = 0;
}Input;


#endif