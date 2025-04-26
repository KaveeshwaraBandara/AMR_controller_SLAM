cmake_minimum_required(VERSION 3.10)
project(SLAM)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

add_executable(slam
    src/main.cpp
    src/Odometry.cpp
)
target_include_directories(slam PRIVATE include)
target_link_libraries(slam PRIVATE ${OpenCV_LIBS})
