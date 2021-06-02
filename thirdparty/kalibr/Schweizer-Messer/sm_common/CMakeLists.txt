cmake_minimum_required(VERSION 2.8.3)
project(sm_common)

find_package(Boost REQUIRED COMPONENTS system)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -D__STRICT_ANSI__")

include_directories(${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/progress_info.cpp
)
target_link_libraries(${PROJECT_NAME}
                      ${Boost_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
        include)

##################
## Installation ##
##################

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${DV_MODULES_DIR}/imu-calibration)
