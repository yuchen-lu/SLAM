
cmake_minimum_required( VERSION 2.8)
# add directories

set(CMAKE_BUILD_TYPE "Debug")
project ( EigenMatrix)
add_executable( EigenMatrix EigenMatrix.cpp)
include_directories("/usr/include/eigen3") #eigen 3 doesnt have tartget_link_libraries
#include_directories("/usr/lib/cmake/eigen3") #eigen 3 doesnt have tartget_link_libraries
#include_directories("/usr/share/doc/libeigen3-dev") #eigen 3 doesnt have tartget_link_libraries
