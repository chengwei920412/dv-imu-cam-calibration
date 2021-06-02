cmake_minimum_required(VERSION 2.8.3)
project(sm_random)

##############
## Building ##
##############

add_definitions(-fPIC)

add_library(${PROJECT_NAME} SHARED
  src/random.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        include)

##################
## Installation ##
##################

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${DV_MODULES_DIR}/imu-calibration)
