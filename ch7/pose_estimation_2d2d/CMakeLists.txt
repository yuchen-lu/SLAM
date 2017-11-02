cmake_minimum_required(VERSION 2.8)
project(pose_estimation_2d2d)



#install(TARGETS pose_estimation_2d2d RUNTIME DESTINATION bin)

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )
add_executable(pose_estimation_2d2d main.cpp)
target_link_libraries( pose_estimation_2d2d ${OpenCV_LIBS})

