cmake_minimum_required(VERSION 2.8.3)
project(sm_logging)

find_package(Boost REQUIRED COMPONENTS system thread regex)

include_directories(${Boost_INCLUDE_DIRS})

add_definitions(-std=c++0x)

##############
## Building ##
##############

add_library(${PROJECT_NAME} 
  src/Logger.cpp
  src/StdOutLogger.cpp
  src/LoggingEvent.cpp
  src/LoggingGlobals.cpp
  src/Formatter.cpp
  src/Tokens.cpp
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
