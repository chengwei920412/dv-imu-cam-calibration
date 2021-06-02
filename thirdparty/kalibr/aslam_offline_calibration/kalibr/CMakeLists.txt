cmake_minimum_required(VERSION 2.8.3)
project(kalibr)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_definitions(-fPIC)

add_library(${PROJECT_NAME}_errorterms SHARED
  src/EuclideanError.cpp
  src/GyroscopeError.cpp
  src/AccelerometerError.cpp
)

target_link_libraries(${PROJECT_NAME}_errorterms
        aslam_backend
        aslam_backend_expressions)
target_include_directories(${PROJECT_NAME}_errorterms PUBLIC
        include)

INSTALL(TARGETS ${PROJECT_NAME}_errorterms DESTINATION ${DV_MODULES_DIR}/imu-calibration)
