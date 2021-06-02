cmake_minimum_required(VERSION 2.8.3)
project(sm_opencv)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

add_definitions(-std=c++0x -D__STRICT_ANSI__)

include_directories(${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
        src/empty.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
        include)
