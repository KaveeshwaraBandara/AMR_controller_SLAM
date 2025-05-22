cmake_minimum_required(VERSION 3.10)
project(SLAM_LIDAR)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
include_directories(
    ${PROJECT_SOURCE_DIR}/rplidar_sdk/sdk/include
    ${PROJECT_SOURCE_DIR}/lidar
    ${PROJECT_SOURCE_DIR}/mapping
    ${PROJECT_SOURCE_DIR}/icp
    ${PROJECT_SOURCE_DIR}/EKF
    ${PROJECT_SOURCE_DIR}/External/eigen
    ${PROJECT_SOURCE_DIR}/imu
    ${PROJECT_SOURCE_DIR}/Communication
    ${PROJECT_SOURCE_DIR}/Astar
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(main
    main.cpp
    lidar/LidarReader.cpp
    mapping/OccupancyGrid.cpp
    icp/ICPMatcher.cpp
    EKF/EKF.cpp
    imu/IMUReader.cpp
    Astar/Astar.cpp
)

target_link_libraries(main
    ${PROJECT_SOURCE_DIR}/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a
    pthread
    ${OpenCV_LIBS}
)

