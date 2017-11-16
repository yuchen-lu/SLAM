cmake_minimum_required(VERSION 2.8)
project(uselk)

#add c++ 11 std support
set (CMAKE_CXX_FLAGS "-std=c++11")
#set (CMAKE_BUILD_TYPE "Debug")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )





#install(TARGETS uselk RUNTIME DESTINATION bin)


add_executable(uselk main.cpp)

target_link_libraries( uselk ${OpenCV_LIBS})