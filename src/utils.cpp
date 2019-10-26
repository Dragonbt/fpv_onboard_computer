#include "utils.hpp"

int64_t intervalMs(high_resolution_clock::time_point end, high_resolution_clock::time_point start)
{
    duration<double> time_span = end - start;
    milliseconds d = duration_cast< milliseconds >( time_span );
    return d.count();
}

string getCurrentTime( void )
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,sizeof(buffer),"%m-%d_%H-%M-%S",timeinfo);
    string current_time(buffer);
    return current_time;
}

void remotePrint( string msg )
{
    if( msg.size() > MAX_MSG_LENGTH )
    {
        cout << "[ERROR]: remote print overflow" << endl;
        return;
    }
    string_mutex.lock();
    if( string_topic.size() > MAX_VEC_SIZE )
    {
        string_topic.clear();
    }
    string_topic.push_back(msg);
    string_mutex.unlock();
    return;
}

void writePositionNED(PositionNED position)
{
    log_file_mutex.lock();
    if( ! log_file_topic.is_open() )
    {
        log_file_topic.open( string("../log/") + getCurrentTime() + string(".csv") );
    }
    log_file_topic << "PositionNED" << ", "
            << position.north_m << ", "
            << position.east_m << ", "
            << position.down_m << ", "
            << position.time_ms << endl;
    log_file_mutex.unlock();
}

void writeVelocityNED(VelocityNED velocity)
{
    log_file_mutex.lock();
    if( ! log_file_topic.is_open() )
    {
        log_file_topic.open( string("../log/") + getCurrentTime() + string(".csv") );
    }
    log_file_topic << "VelocityNED" << ", "
            << velocity.north_m_s << ", "
            << velocity.east_m_s << ", "
            << velocity.down_m_s << ", "
            << velocity.time_ms << endl;
    log_file_mutex.unlock();
}

void writeAttitude(EulerAngle euler_angle)
{
    log_file_mutex.lock();
    if( ! log_file_topic.is_open() )
    {
        log_file_topic.open( string("../log/") + getCurrentTime() + string(".csv") );
    }
    log_file_topic << "EulerAngle" << ", "
            << euler_angle.roll_deg << ", "
            << euler_angle.pitch_deg << ", "
            << euler_angle.yaw_deg << ", "
            << euler_angle.time_ms << endl;
    log_file_mutex.unlock();
}

void writeInputAttitude( InputAttitude input_attitude )
{
    log_file_mutex.lock();
    if( ! log_file_topic.is_open() )
    {
        log_file_topic.open( string("../log/") + getCurrentTime() + string(".csv") );
    }
    log_file_topic << "InputAttitude" << ", "
            << input_attitude.roll_deg << ", "
            << input_attitude.pitch_deg << ", "
            << input_attitude.yaw_deg << ", "
            << input_attitude.thrust << ", "
            << input_attitude.time_ms << endl;
    log_file_mutex.unlock();
}

void writeReference( Reference reference)
{
    log_file_mutex.lock();
    if( ! log_file_topic.is_open() )
    {
        log_file_topic.open( string("../log/") + getCurrentTime() + string(".csv") );
    }
    log_file_topic << "Reference" << ", "
            << reference.down_m << ", "
            << reference.time_ms << endl;
    log_file_mutex.unlock();
}