cmake_minimum_required(VERSION 3.5)
project(rotate_panorama_video)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(rotate_panorama_video rotate_panorama_video.cpp)
target_link_libraries(rotate_panorama_video ${OpenCV_LIBS})
