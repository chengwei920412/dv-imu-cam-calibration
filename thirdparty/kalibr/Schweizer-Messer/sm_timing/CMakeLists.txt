cmake_minimum_required(VERSION 2.8.3)
project(sm_timing)

find_package(Boost REQUIRED COMPONENTS system)

add_definitions(-std=c++0x -D__STRICT_ANSI__)

include_directories(${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

add_library(${PROJECT_NAME}
  src/Timer.cpp
  src/NsecTimeUtilities.cpp
)
target_link_libraries(${PROJECT_NAME} 
                      ${Boost_LIBRARIES}
                      sm_common
                      sm_random)
target_include_directories(${PROJECT_NAME} PUBLIC
        include)