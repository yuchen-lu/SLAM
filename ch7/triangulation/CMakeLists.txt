cmake_minimum_required(VERSION 2.6)
project(triangulation)

add_executable(triangulation main.cpp)

install(TARGETS triangulation RUNTIME DESTINATION bin)
