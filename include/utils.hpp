#ifndef _CLOCKS_
#define _CLOCKS_

#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>
#include "struct.hpp"
#include "control_node.hpp"
#include "protocol.hpp"
#include <fstream>

using namespace std::chrono;
using namespace std;

extern vector<string> string_topic;
extern mutex string_mutex;

extern ofstream log_file_topic;
extern mutex log_file_mutex;

int64_t intervalMs(high_resolution_clock::time_point end, high_resolution_clock::time_point start);

string getCurrentTime( void );

void remotePrint( string msg );

void writePositionNED(PositionNED position);
void writeVelocityNED(VelocityNED velocity);
void writeAttitude(EulerAngle attitude);
void writeInputAttitude(InputAttitude input_attitude);
void writeReference( Reference reference);
#endif