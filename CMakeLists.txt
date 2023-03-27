cmake_minimum_required(VERSION 3.5)
project(rotate_panorama_video)

# Use C++14 standard and enable compiler optimization
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenCV REQUIRED)

add_executable(rotate_panorama_video rotate_panorama_video.cpp)
target_link_libraries(rotate_panorama_video ${OpenCV_LIBS})