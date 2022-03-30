add_library(kalibr_imu_camera_calibration SHARED
        common.cpp)
target_link_libraries(kalibr_imu_camera_calibration
        aslam_backend
        aslam_cameras
        aslam_cv_backend
        aslam_splines
        bsplines
        incremental_calibration
        sm_kinematics
        kalibr_errorterms
        aslam_cv_error_terms
        ${DV_LIBRARIES}
        )
target_include_directories(kalibr_imu_camera_calibration PUBLIC
        ../../include/dv-imu-cam-calibration)

INSTALL(TARGETS kalibr_imu_camera_calibration DESTINATION ${DV_MODULES_DIR}/imu-calibration)