cmake_minimum_required(VERSION 2.8.3)
project(sm_opencv)

find_package(Boost REQUIRED COMPONENTS system)

add_library(${PROJECT_NAME}
        src/empty.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
        include)

add_definitions(-std=c++0x -D__STRICT_ANSI__)

include_directories(${Boost_INCLUDE_DIRS})
