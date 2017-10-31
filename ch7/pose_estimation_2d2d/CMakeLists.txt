cmake_minimum_required(VERSION 2.8)
project(pose_estimation_2d2d)

add_executable(pose_estimation_2d2d main.cpp)

install(TARGETS pose_estimation_2d2d RUNTIME DESTINATION bin)
