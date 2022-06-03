# Init CMake (require at least version 3.16)
CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

PROJECT(dv-imu-cam-calibration
	VERSION 1.1.0
	LANGUAGES C CXX)

ADD_DEFINITIONS(-march=native)

# Define installation paths
INCLUDE(GNUInstallDirs)

# Basic setup
FIND_PACKAGE(dv 1.5.2 REQUIRED)

SET(CMAKE_C_VISIBILITY_PRESET default)
SET(CMAKE_CXX_VISIBILITY_PRESET default)

# Set full RPATH, modules are libraries for DV
SET(CMAKE_INSTALL_RPATH ${USER_LOCAL_PREFIX}/${DV_MODULES_DIR}/imu-calibration)

add_subdirectory(src)
add_subdirectory(thirdparty)
