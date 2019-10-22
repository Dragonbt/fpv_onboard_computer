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
    string_vec_mutex.lock();
    if( string_vec_topic.size() > MAX_VEC_SIZE )
    {
        string_vec_topic.clear();
    }
    string_vec_topic.push_back(msg);
    string_vec_mutex.unlock();
    return;
}