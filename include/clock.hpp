#ifndef _CLOCKS_
#define _CLOCKS_

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std;

double intervalMs(high_resolution_clock::time_point end, high_resolution_clock::time_point start);

string getCurrentTime( void );

#endif