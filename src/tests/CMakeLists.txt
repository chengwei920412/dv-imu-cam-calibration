
add_executable(test_icc_calibrator
        testIccCalibrator.cpp)
target_link_libraries(test_icc_calibrator
        gtest
        pthread
        )