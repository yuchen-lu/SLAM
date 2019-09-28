cmake_minimum_required(VERSION 3.9)
project(ch8)

set(CMAKE_CXX_STANDARD 11)

add_executable(ch8
        directMethod/direct_sparse/main.cpp
        useLk/main.cpp)
