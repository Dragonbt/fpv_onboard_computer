#ifndef _CLOCKS_
#define _CLOCKS_

#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>
#include "struct.hpp"

#include "const.hpp"
#include <fstream>
#include "topic.hpp"

using namespace std::chrono;
using namespace std;

extern high_resolution_clock::time_point init_timepoint;
extern Topic<string> string_topic;

float limit_values(float values, float min_values, float max_values);

float deg2rad(float deg);

float rad2deg(float rad);

int64_t intervalMs(high_resolution_clock::time_point end, high_resolution_clock::time_point start);

int64_t timestampf( void );

string getCurrentTime( void );

void remotePrint( string msg );

#endif