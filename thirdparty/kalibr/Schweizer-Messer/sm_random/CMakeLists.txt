cmake_minimum_required(VERSION 2.8.3)
project(sm_random)

include_directories(include)

##############
## Building ##
##############

add_library(${PROJECT_NAME}
  src/random.cpp
)
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