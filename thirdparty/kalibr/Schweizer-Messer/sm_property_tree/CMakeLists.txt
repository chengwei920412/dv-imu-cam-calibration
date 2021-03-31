cmake_minimum_required(VERSION 2.8.3)
project(sm_property_tree)

find_package(Boost REQUIRED COMPONENTS system filesystem)

add_definitions(-D__STRICT_ANSI__)
remove_definitions(-std=c++0x -std-c++11)

add_definitions(-fPIC)

include_directories(${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

add_library(${PROJECT_NAME} SHARED
  src/PropertyTree.cpp
  src/BoostPropertyTreeImplementation.cpp
  src/BoostPropertyTree.cpp
  src/PropertyTreeImplementation.cpp
  src/BoostPropertyTreeSupport.cpp
)

target_link_libraries(${PROJECT_NAME} 
                      ${Boost_LIBRARIES}
                      pthread
                      sm_common)

target_include_directories(${PROJECT_NAME} PUBLIC
        include)
