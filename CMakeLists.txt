cmake_minimum_required(VERSION 2.8)
project(onboard)

SET(CMAKE_CXX_FLAGS "-std=c++0x")
set (PLATFORM_LIBS pthread ${CMAKE_DL_LIBS})

find_package(OpenCV REQUIRED)

include_directories(include) 

FILE(GLOB_RECURSE SRC "src/*.cpp" )

add_executable( onboard ${SRC})
target_link_libraries( onboard ${OpenCV_LIBS} ${PLATFORM_LIBS} )
