
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

add_library(calibrator
        calibrator.cpp)
target_link_libraries(calibrator
        ${OpenCV_LIBS}
        aslam_cameras)
target_include_directories(calibrator PUBLIC
        ../../include/dv-imu-cam-calibration)
