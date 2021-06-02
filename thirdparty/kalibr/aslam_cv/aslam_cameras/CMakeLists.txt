cmake_minimum_required(VERSION 2.8.3)
project(aslam_cameras)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

ADD_DEFINITIONS(-DASLAM_USE_ROS )

add_definitions(-fPIC)

find_package(Boost REQUIRED COMPONENTS system serialization filesystem)

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} SHARED
  src/CameraGeometryBase.cpp
  src/GlobalShutter.cpp
  src/RollingShutter.cpp
  src/NoMask.cpp
  src/NoDistortion.cpp
  src/RadialTangentialDistortion.cpp
  src/EquidistantDistortion.cpp
  src/FovDistortion.cpp
  src/ImageMask.cpp
  src/GridCalibrationTargetObservation.cpp
  src/GridCalibrationTargetBase.cpp
  src/GridCalibrationTargetCheckerboard.cpp
  src/GridCalibrationTargetCirclegrid.cpp
  src/GridDetector.cpp
  src/Triangulation.cpp
  src/FrameBase.cpp
  src/KeypointBase.cpp
  src/DescriptorBase.cpp
  src/BackProjection.cpp
  src/Landmark.cpp
  src/Image.cpp
  )

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
		${OpenCV_LIBS}
        sm_boost
        sm_eigen
        sm_kinematics
        sm_logging
        sm_opencv
        sm_property_tree
        aslam_time)

target_include_directories(${PROJECT_NAME} PUBLIC
        include)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${DV_MODULES_DIR}/imu-calibration)
