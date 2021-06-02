cmake_minimum_required(VERSION 2.8.3)
project(bsplines)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_definitions(-fPIC)

add_library(${PROJECT_NAME} SHARED
  src/BSpline.cpp
  src/BSplinePose.cpp
)
target_link_libraries(${PROJECT_NAME}
        sparse_block_matrix
        sm_kinematics)
target_include_directories(${PROJECT_NAME} PUBLIC
        include)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${DV_MODULES_DIR}/imu-calibration)
