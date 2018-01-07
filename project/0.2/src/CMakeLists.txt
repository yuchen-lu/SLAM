add_library(myslam SHARED frame.cpp map.cpp
        camera.cpp config.cpp visual_odometry.cpp mappoint.cpp visual_odometry.cpp mappoint.cpp)

target_link_libraries(myslam ${THIRD_PARTY_LIBS})
