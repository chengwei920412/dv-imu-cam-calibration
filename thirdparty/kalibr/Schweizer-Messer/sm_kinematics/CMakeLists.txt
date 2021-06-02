cmake_minimum_required(VERSION 2.8.3)
project(sm_kinematics)

find_package(Boost REQUIRED COMPONENTS system serialization filesystem)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")

add_definitions(-fPIC)

include_directories(${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

add_library(${PROJECT_NAME} SHARED
  src/quaternion_algebra.cpp
  src/rotations.cpp
  src/transformations.cpp
  src/RotationalKinematics.cpp
  src/RotationVector.cpp
  src/EulerAnglesZYX.cpp
  src/EulerAnglesYawPitchRoll.cpp
  src/EulerRodriguez.cpp
  src/Transformation.cpp
  src/homogeneous_coordinates.cpp
  src/HomogeneousPoint.cpp
  src/UncertainTransformation.cpp
  src/UncertainHomogeneousPoint.cpp
  src/three_point_methods.cpp
  src/EulerAnglesZXY.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        sm_boost
        sm_common
        sm_eigen)

target_include_directories(${PROJECT_NAME} PUBLIC
        include)

##################
## Installation ##
##################

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${DV_MODULES_DIR}/imu-calibration)
