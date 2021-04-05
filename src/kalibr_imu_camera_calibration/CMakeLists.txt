
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_definitions(-fPIC)

set(CMAKE_CXX_STANDARD 17)

add_library(kalibr_imu_camera_calibration SHARED
        iccCalibrator.cpp
        iccSensors.cpp)
target_link_libraries(kalibr_imu_camera_calibration
        aslam_backend
        aslam_cameras
        aslam_cv_backend
        this is bad aslam_cv_error_terms
        aslam_splines
        bsplines
        config_reader
        incremental_calibration
        sm_kinematics
        kalibr_errorterms
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        )
target_include_directories(kalibr_imu_camera_calibration PUBLIC
        ../../include/dv-imu-cam-calibration)

