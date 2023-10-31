cmake_minimum_required(VERSION 3.26)
project(08_opencv)

set(CMAKE_CXX_STANDARD 17)
set(OpenCV_DIR /opt/homebrew/Cellar/opencv/4.8.1_1) #设置你的 build_opencv 路径

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(08_opencv main.cpp src/MyOpenCV.cpp)

target_link_libraries(08_opencv ${OpenCV_LIBS})