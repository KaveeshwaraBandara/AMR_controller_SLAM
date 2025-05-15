cmake_minimum_required(VERSION 3.10)
project(SLAM_LIDAR)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


include_directories(
    ${PROJECT_SOURCE_DIR}/rplidar_sdk/sdk/include
    ${PROJECT_SOURCE_DIR}/lidar
    ${PROJECT_SOURCE_DIR}/mapping
)

add_executable(main
    main.cpp
    lidar/LidarReader.cpp
    mapping/OccupancyGrid.cpp
)

target_link_libraries(main
    ${PROJECT_SOURCE_DIR}/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a
    pthread
)

