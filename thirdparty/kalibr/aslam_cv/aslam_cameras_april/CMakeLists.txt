cmake_minimum_required(VERSION 2.8.3)
project(aslam_cameras_april)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_definitions(-fPIC)
ADD_DEFINITIONS(-DASLAM_USE_ROS )


add_library(${PROJECT_NAME} SHARED
  src/GridCalibrationTargetAprilgrid.cpp
  )

find_package(Boost REQUIRED COMPONENTS serialization system)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        aslam_cameras
        ethz_apriltag2
        sm_common
        sm_logging
        )
target_include_directories(${PROJECT_NAME} PUBLIC
        include)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${DV_MODULES_DIR}/imu-calibration)
