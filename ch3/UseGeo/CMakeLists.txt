
cmake_minimum_required( VERSION 2.8)
# add directories

set(CMAKE_BUILD_TYPE "Debug")

project (UseGeo)
add_executable( UseGeo UseGeometry.cpp)

include_directories("/usr/include/eigen3") #eigen 3 doesnt have tartget_link_libraries

