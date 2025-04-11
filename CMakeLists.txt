cmake_minimum_required(VERSION 3.10)
project(SLAM)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

add_executable(slam main.cpp)
target_link_libraries(slam ${OpenCV_LIBS})
