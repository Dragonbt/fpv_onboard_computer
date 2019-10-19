cmake_minimum_required(VERSION 2.8)
project(onboard)

SET(CMAKE_CXX_FLAGS "-std=c++0x")
set (PLATFORM_LIBS pthread ${CMAKE_DL_LIBS})
add_definitions("-std=c++11 -Wall -Wextra -Werror -Wno-psabi")
find_package(OpenCV REQUIRED)
find_package(MAVSDK REQUIRED)

include_directories(include) 

FILE(GLOB_RECURSE SRC "src/*.cpp" )

add_executable( onboard ${SRC})
target_link_libraries( onboard ${OpenCV_LIBS} ${PLATFORM_LIBS} 
                        MAVSDK::mavsdk_action
                        MAVSDK::mavsdk_offboard
                        MAVSDK::mavsdk_telemetry
                        MAVSDK::mavsdk)
