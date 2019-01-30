cmake_minimum_required(VERSION 3.12)
project(jacob)

set(CMAKE_CXX_STANDARD 11)

add_executable(jacob main.cpp)

target_include_directories(jacob PUBLIC
        ../Sophus/py
        ../eigen3
        /usr/include/eigen3
        ../ModernRobotics/packages/Python
        )

