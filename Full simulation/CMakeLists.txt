cmake_minimum_required(VERSION 3.10)
project(SLAM_LIDAR_SIM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)

# Gazebo Transport and Messages (adjust versions based on installed system)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/mapping
    ${PROJECT_SOURCE_DIR}/icp
    ${PROJECT_SOURCE_DIR}/EKF
    ${PROJECT_SOURCE_DIR}/Communication
    ${PROJECT_SOURCE_DIR}/Astar
    ${PROJECT_SOURCE_DIR}/External/eigen
    ${OpenCV_INCLUDE_DIRS}
    ${gz-msgs10_INCLUDE_DIRS}     # Ensure message headers like laserscan.pb.h are found
    ${gz-transport13_INCLUDE_DIRS}
)

# Add executable and source files
add_executable(main
    main.cpp
    mapping/OccupancyGrid.cpp
    icp/ICPMatcher.cpp
    EKF/EKF.cpp
    Astar/Astar.cpp
)

# Link required libraries
target_link_libraries(main
    ${OpenCV_LIBS}
    protobuf::libprotobuf
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    pthread
)

