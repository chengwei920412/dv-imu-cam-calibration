
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_library(icc_calibrator
        iccCalibrator.cpp)
target_link_libraries(icc_calibrator
        aslam_cameras
        aslam_splines
        bsplines
        incremental_calibration)
target_include_directories(icc_calibrator PUBLIC
        ../../include/dv-imu-cam-calibration)

