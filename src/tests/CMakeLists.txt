

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_executable(test_calibrator
        testCalibrator.cpp)
target_link_libraries(test_calibrator
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        aslam_backend
        aslam_cameras
        aslam_cv_backend
        aslam_splines
        bsplines
        calibrator
        incremental_calibration
        sm_boost
        sm_kinematics
        kalibr_errorterms
        kalibr_imu_camera_calibration
        gtest
        pthread
        )