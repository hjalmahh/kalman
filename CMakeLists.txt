cmake_minimum_required(VERSION 3.29)
project(sensor_fusion)

set(CMAKE_CXX_STANDARD 20)

find_package(Eigen3 CONFIG REQUIRED)

add_executable(sensor_fusion
        src/main.cpp
        src/kalmanFilter.cpp
        include/laser_ros.h
        src/laser_ros.cpp
        include/line_detect.h
        src/line_detect.cpp
        include/struct.h
        #src/main_line.cpp
)

target_link_libraries(sensor_fusion
        PRIVATE Eigen3::Eigen
)

target_include_directories(sensor_fusion
        PRIVATE ${PROJECT_SOURCE_DIR}/include
)