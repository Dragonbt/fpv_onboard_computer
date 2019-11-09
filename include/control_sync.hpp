#ifndef _CONTROL_SYNC_
#define _CONTROL_SYNC_

#include <thread>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <opencv2/core/core.hpp>
#include "utils.hpp"
#include "struct.hpp"

using namespace std;
using namespace mavsdk;
using namespace chrono;
using namespace cv;

extern Topic<InputAttitude> input_attitude_topic;

extern Topic<float> down_reference_topic;

extern Topic<Vector2f> ne_reference_topic;

extern Topic<Vector2f> pos_err_xy_topic;

void healthCheck( shared_ptr<Telemetry> telemetry );
void waitForArmed( shared_ptr<Telemetry> telemetry );
void quitOffboard( shared_ptr<Offboard> offboard );

void offbCtrlAttitude(shared_ptr<Offboard> offboard, Offboard::Attitude attitude);
void offbCtrlPositionNED( std::shared_ptr<mavsdk::Offboard> offboard, Offboard::PositionNEDYaw position );

class AltitudeThrustControl{
    public:
    void reset(FileNode altitude_pid);
    float downOffset( float err, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms );
    float down( float pos_sp_z, PositionNED, VelocityBody vel_body, EulerAngle attitude, int dt_ms );
    float hold( PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms );
    float landing();

    private:
    float Kp_z, Ki_z, Kd_z;
    float time_change;
    float pos_z, vel_z, roll, pitch;
    float pos_sp_z;
    float err_pos_z;
    float int_pos_z = 0;
    float offset_thrust;
    float thrust_desired_D;
    float mid_thrust = 0.55;
    float uMax = mid_thrust + 0.1f;
    float uMin = mid_thrust - 0.1f;
    bool stop_integral_D = false;
    int sign_thr_int_z;
    int times = 0;
    float calcThrust();
};

class VisionRollThrustControl{
    public:
    void reset(FileNode vision_pid, FileNode altitude_pid);
    void angleOffset( float& roll_deg, float& thrust, DetectionResult target, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms );
    void hold(float& roll_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms);

    private:
    AltitudeThrustControl altitude_thrust_control;
    float y_deg, z_deg;
    float Ky, Kz;
    float Kp_y, Ki_y, Kd_y;
    float err_pos_z = 0.0f;
    float err_pos_y = 0.0f;
    float int_pos_y = 0.0f;
    float vel_err = 0.0f;
    float tilt_max = P_I / 4;
    float thrust_max = 1.0f;
    float thrust_desired_y;
    float alt_thrust;
    float time_change;

    float vel_y;
    float vel_sp_y = 0.0f;
    float vel_err_y = 0.0f;
    float pos_sp_z;
    float calcRoll();
};

class FlowPosThrustControl {
public:
	void reset(FileNode flow_pid, FileNode altitude_pid);
    void positionBodyOffset( float& roll_deg, float& pitch_deg, float& thrust, Vector3f offset_body, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms);
    void hold(float& roll_deg, float& pitch_deg, float& thrust, PositionNED pos_ned, VelocityBody vel_body, EulerAngle attitude, int dt_ms);
	void calcRollPitchThrust(float& roll_deg, float& pitch_deg, float& thrust);
private:
	AltitudeThrustControl altitude_thrust_control;
	Vector2f int_pos_xy = { 0.0f,0.0f };
    Vector2f pos_sp_ne;
    float pos_sp_d;
    Vector2f pos_err_ne;
    Vector2f pos_err_xy;
    Vector2f vel_err_xy;
	float Kp_x, Kp_y, Ki_x, Ki_y, Kd_x, Kd_y;
    Vector2f Kp, Ki, Kd;
    float alt_thrust;
    float tilt_max = P_I / 4;
    float thrust_max = 1.0f;
    float time_change;
};
#endif