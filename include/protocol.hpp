#ifndef _PROTOCOL_
#define _PROTOCOL_

#define MAX_MSG_LENGTH 0x22FF
#define HEAD 0xAAAA
#define TAIL 0xDDDD

//send msg type
#define HEART_BEAT_MSG 0
#define IMG_MSG 1
#define POSITION_MSG 2
#define VELOCITY_MSG 3
#define ATTITUDE_MSG 4
#define INPUT_ATTITUDE_MSG 5
#define STATUS_MSG 6
#define LOG_MSG 7
#define REFERENCE_MSG 8
#define TARGET_MSG 9
#define POSITION_BODY_MSG 10

#define MISSION_COMMAND_MSG 15

#define SAFE_QUIT_COMMAND -2
#define FORCE_QUIT_COMMAND -1
#define STEP_COMMAND 0
#define FLOW_HOLD_COMMAND 2
#define VISION_HOLD_COMMAND 4

#endif