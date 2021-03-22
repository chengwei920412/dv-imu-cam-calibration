cmake_minimum_required(VERSION 2.8.9)
project (hello)

add_subdirectory(thirdparty)

add_library(hi SHARED hi.hpp hi.cpp)

add_executable(hello helloworld.cpp)
target_link_libraries(hello hi)
