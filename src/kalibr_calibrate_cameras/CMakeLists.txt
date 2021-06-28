
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_library(camera_calibrator SHARED
        CameraCalibrator.cpp)
target_link_libraries(camera_calibrator
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        aslam_backend
        aslam_cameras
        aslam_cameras_april
        aslam_cv_backend
        aslam_splines
        incremental_calibration
        aslam_cv_error_terms
        kalibr_errorterms
        )
target_include_directories(camera_calibrator PUBLIC
        ../../include/dv-imu-cam-calibration)

INSTALL(TARGETS camera_calibrator DESTINATION ${DV_MODULES_DIR}/imu-calibration)
