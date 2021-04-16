cmake_minimum_required(VERSION 2.8)

project(ethz_apriltag2)


find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)

find_package(OpenCV REQUIRED)

add_definitions(-fPIC -O3)

#library
file(GLOB SOURCE_FILES "src/*.cc")
add_library(${PROJECT_NAME} SHARED
        ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        )
target_include_directories(${PROJECT_NAME} PUBLIC
        include)