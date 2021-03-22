cmake_minimum_required(VERSION 2.8.3)
project(sm_boost)

find_package(Boost REQUIRED COMPONENTS system serialization thread)

include_directories(${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

add_library(${PROJECT_NAME}
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

