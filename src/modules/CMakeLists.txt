
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

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
        camera_calibrator
        config_reader
        incremental_calibration
        kalibr_errorterms
        kalibr_imu_camera_calibration
        sm_boost
        sm_kinematics
        )

INSTALL(TARGETS imu-cam-calibration DESTINATION ${DV_MODULES_DIR}/imu-calibration)
