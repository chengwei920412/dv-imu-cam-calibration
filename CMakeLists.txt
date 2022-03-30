# Init CMake (require at least version 3.10.0)
CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

# General build settings
IF (NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Possible build types: None Debug Release RelWithDebInfo MinSizeRel")
ENDIF()

IF (NOT CMAKE_INSTALL_PREFIX)
	IF (UNIX AND NOT APPLE)
		SET(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "CMake default install prefix, set to /usr on Unix/Linux")
	ELSEIF(APPLE)
		SET(CMAKE_INSTALL_PREFIX "/usr/local" CACHE STRING "CMake default install prefix, set to /usr/local on macOS")
	ELSE()
		MESSAGE(FATAL_ERROR "CMAKE_INSTALL_PREFIX is not set")
	ENDIF()
ENDIF()

PROJECT(dv-imu-cam-calibration
	VERSION 1.0.0
	LANGUAGES C CXX)

ADD_DEFINITIONS(-march=native)

# Define installation paths
INCLUDE(GNUInstallDirs)

# Custom CMake base module
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
#	${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/dv ${CMAKE_INSTALL_PREFIX}/share/dv
#	/usr/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/share/dv
#	/usr/local/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/local/share/dv)
#
## Support new Homebrew paths on MacOS
#IF (APPLE)
#	SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
#		/opt/homebrew/${CMAKE_INSTALL_DATAROOTDIR}/dv /opt/homebrew/share/dv)
#ENDIF()

# Basic setup
FIND_PACKAGE(dv 1.5.0 REQUIRED)
SET(DV_LIBRARIES dv::sdk)

SET(CMAKE_C_VISIBILITY_PRESET default)
SET(CMAKE_CXX_VISIBILITY_PRESET default)

# Set full RPATH, modules are libraries for DV
SET(CMAKE_INSTALL_RPATH ${USER_LOCAL_PREFIX}/${DV_MODULES_DIR}/imu-calibration)

add_subdirectory(src)
add_subdirectory(thirdparty)
