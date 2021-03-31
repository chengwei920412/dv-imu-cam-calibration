
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_definitions(-fPIC)

add_library(calibrator SHARED
        calibrator.cpp)
target_link_libraries(calibrator
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        aslam_backend
        aslam_cameras
        aslam_splines
        bsplines
        incremental_calibration
        sm_boost
        sm_kinematics
        kalibr_errorterms)
target_include_directories(calibrator PUBLIC
        ../../include/dv-imu-cam-calibration)
