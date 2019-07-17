
cmake_minimum_required( VERSION 2.8)
# add directories

set(CMAKE_BUILD_TYPE "Debug")

project (practice1)
add_executable( practice1 practice1.cpp ../practice1/practice1.cpp)

include_directories("/usr/include/eigen3") #eigen 3 doesnt have tartget_link_libraries

