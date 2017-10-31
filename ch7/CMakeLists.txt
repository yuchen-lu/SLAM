cmake_minimum_required(VERSION 2.8)
project(feature_extraction)

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )
add_executable( feature_extraction feature_extraction.cpp )
target_link_libraries( feature_extraction ${OpenCV_LIBS})

