# Init CMake (require at least version 3.10.0)
CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

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

add_definitions(-fPIC)

# Project name and version
PROJECT(dv-imu-cam-calibration-cpp
        VERSION 1.0.0
        LANGUAGES C CXX)

# Define installation paths
INCLUDE(GNUInstallDirs)

# Custom CMake base module
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/dv ${CMAKE_INSTALL_PREFIX}/share/dv
        /usr/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/share/dv
        /usr/local/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/local/share/dv)

# Support new Homebrew paths on MacOS
IF (APPLE)
    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
            /opt/homebrew/${CMAKE_INSTALL_DATAROOTDIR}/dv /opt/homebrew/share/dv)
ENDIF()

# Basic setup
INCLUDE(dv-modules)
DV_MODULE_SETUP()

# Set full RPATH, modules are libraries for DV
SET(CMAKE_INSTALL_RPATH ${DV_MODULES_DIR})

# Compile this module (standard C++ file, no additional dependencies)
ADD_LIBRARY(imu-cam-calibration SHARED
        ImuCamCalibration.cpp)

SET_TARGET_PROPERTIES(imu-cam-calibration
        PROPERTIES
        PREFIX "dv_"
        )

TARGET_LINK_LIBRARIES(imu-cam-calibration PRIVATE
        ${DV_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        aslam_backend
        aslam_cameras
        aslam_cameras_april
        aslam_splines
        bsplines
        calibrator
        config_reader
        incremental_calibration
        kalibr_errorterms
        kalibr_imu_camera_calibration
        sm_boost
        sm_kinematics
        )

INSTALL(TARGETS imu-cam-calibration DESTINATION ${DV_MODULES_DIR})
