
cmake_minimum_required( VERSION 2.8)
# add directories

set(CMAKE_BUILD_TYPE "Debug")

project ( UseGeo)

include_directories("/usr/include/eigen3") #eigen 3 doesnt have tartget_link_libraries

add_executable( UseGeometry UseGeometry.cpp)
