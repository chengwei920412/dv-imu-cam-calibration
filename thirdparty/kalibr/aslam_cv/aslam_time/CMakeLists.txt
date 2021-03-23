cmake_minimum_required(VERSION 2.8.3)
project(aslam_time)

find_package(Boost REQUIRED COMPONENTS system)

add_library(${PROJECT_NAME}
    src/time.cpp
    src/duration.cpp)

set(OTHER_LIBS)
if(NOT APPLE)
  set(OTHER_LIBS rt)
endif()

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${OTHER_LIBS}
        )
target_include_directories(${PROJECT_NAME} PUBLIC
        include)
