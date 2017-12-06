cmake_minimum_required(VERSION 2.8)
project(triangulation)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )


add_executable(triangulation main.cpp)



target_link_libraries( triangulation ${OpenCV_LIBS})
