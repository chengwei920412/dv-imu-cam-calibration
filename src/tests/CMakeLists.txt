# Unit testing support.
OPTION(ENABLE_TESTS "Build unit tests" ON)

IF (NOT ENABLE_TESTS)
	SET(ENABLE_TESTS 0 CACHE BOOL "Compiles tests using Google's GTest framework.")
	RETURN()
ENDIF()

find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(test_calibrator
		testCalibrator.cpp)

target_link_libraries(test_calibrator
		${DV_LIBRARIES}
		${OpenCV_LIBS}
		${Boost_LIBRARIES}
		aslam_backend
		aslam_cameras
		aslam_cameras_april
		aslam_splines
		bsplines
		incremental_calibration
		kalibr_errorterms
		kalibr_imu_camera_calibration
		sm_boost
		sm_kinematics
		gtest
		pthread
		)

INSTALL(TARGETS test_calibrator DESTINATION ${DV_MODULES_DIR}/imu-calibration)
