
cmake_minimum_required( VERSION 2.8)
# add directories

set(CMAKE_BUILD_TYPE "Debug")

project (UseSophus)

include_directories("/usr/include/eigen3") #eigen 3 doesnt have tartget_link_libraries

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable( UseSophus UseSophus.cpp)

target_link_libraries( UseSophus ${Sophus_LIBRARIES} )


