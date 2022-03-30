IF (NOT ENABLE_TESTS)
	SET(ENABLE_TESTS 0 CACHE BOOL "Compiles tests using Google's GTest framework.")
	RETURN()
ENDIF()
find_package(GTest REQUIRED)

add_executable(test_calibrator
		testCalibrator.cpp)
target_link_libraries(test_calibrator
		${DV_LIBRARIES}
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

#INSTALL(TARGETS test_calibrator DESTINATION ${DV_MODULES_DIR}/imu-calibration)
