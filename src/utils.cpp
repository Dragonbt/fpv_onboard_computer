#include "utils.hpp"

int64_t intervalMs(high_resolution_clock::time_point end, high_resolution_clock::time_point start)
{
    duration<double> time_span = end - start;
    milliseconds d = duration_cast< milliseconds >( time_span );
    return d.count();
}

int64_t timestampf( void )
{
    return intervalMs(high_resolution_clock::now(), init_timepoint);
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
    if( msg.size() > 500 )
    {
        cout << "[ERROR]: remote print overflow" << endl;
        return;
    }
    string_topic.update(msg);
    return;
}

float limit_values(float values, float min_values, float max_values) {
	return values > min_values ? (values < max_values ? values : max_values) : min_values;
}

float deg2rad(float deg) {
	return deg * P_I / 180.0f;
}

float rad2deg(float rad)
{
	return rad * 180.0f / P_I;
}