cmake_minimum_required(VERSION 2.6)
project(direct_sparse)

add_executable(direct_sparse main.cpp)

install(TARGETS direct_sparse RUNTIME DESTINATION bin)
