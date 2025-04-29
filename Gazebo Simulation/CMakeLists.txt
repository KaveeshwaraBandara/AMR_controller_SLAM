cmake_minimum_required(VERSION 3.5)

project(tugbot_lidar_2d_map)

find_package(gz-transport13 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(lidar_2d_mapper src/main.cpp)

target_link_libraries(lidar_2d_mapper 
    gz-transport13::gz-transport13 
    protobuf::libprotobuf 
    ${OpenCV_LIBS}
)

