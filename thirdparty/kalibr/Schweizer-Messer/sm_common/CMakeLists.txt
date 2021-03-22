cmake_minimum_required(VERSION 2.8.3)
project(sm_common)

find_package(Boost REQUIRED COMPONENTS system)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -D__STRICT_ANSI__")

include_directories(include ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/progress_info.cpp
)
target_link_libraries(${PROJECT_NAME} 
                      ${Boost_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
        include)

##################
## Installation ##
##################

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

