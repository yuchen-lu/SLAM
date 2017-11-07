cmake_minimum_required(VERSION 3.5)

project( joinMap )
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

#opencv

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

#EIGEN
include_directories("/usr/include/eigen3/")


#pcl
find_package(PCL REQUIRED COMPONENT common io )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES})

