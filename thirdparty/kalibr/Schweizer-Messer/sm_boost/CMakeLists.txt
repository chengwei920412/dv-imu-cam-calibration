cmake_minimum_required(VERSION 2.8.3)
project(sm_boost)

find_package(Boost REQUIRED COMPONENTS system serialization thread)

include_directories(${Boost_INCLUDE_DIRS})

add_definitions(-fPIC)

##############
## Building ##
##############

add_library(${PROJECT_NAME} SHARED
  src/JobQueue.cpp
  src/portable_binary_iarchive.cpp
  src/portable_binary_oarchive.cpp
)

target_link_libraries(${PROJECT_NAME}
                      ${Boost_LIBRARIES}
                      sm_common)

target_include_directories(${PROJECT_NAME} PUBLIC
        include)

##################
## Installation ##
##################

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${DV_MODULES_DIR}/imu-calibration)
