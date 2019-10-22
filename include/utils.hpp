#ifndef _CLOCKS_
#define _CLOCKS_

#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>
#include "struct.hpp"

using namespace std::chrono;
using namespace std;

extern vector<string> string_vec_topic;
extern mutex string_vec_mutex;

int64_t intervalMs(high_resolution_clock::time_point end, high_resolution_clock::time_point start);

string getCurrentTime( void );

void remotePrint( string msg );

#endif