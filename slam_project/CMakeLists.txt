cmake_minimum_required(VERSION 3.10)
project(SLAM_LIDAR)

set(CMAKE_CXX_STANDARD 11)

include_directories(
    ${PROJECT_SOURCE_DIR}/rplidar_sdk/sdk/include
    ${PROJECT_SOURCE_DIR}/lidar
)

add_executable(main
    main.cpp
    lidar/LidarReader.cpp
)

target_link_libraries(main
    ${PROJECT_SOURCE_DIR}/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a
    pthread
)

