cmake_minimum_required(VERSION 2.8.3)
project(sm_eigen)

find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

include_directories(include
        ${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

add_library(${PROJECT_NAME}
  src/random.cpp
)

target_link_libraries(${PROJECT_NAME} 
                      ${Boost_LIBRARIES}
                      sm_common
                      sm_random)

##################
## Installation ##
##################

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(DIRECTORY include/sm/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
