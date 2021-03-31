
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_library(config_reader SHARED
        ConfigReader.cpp)
target_link_libraries(config_reader
        aslam_cameras)
target_include_directories(config_reader PUBLIC
        ../../include/dv-imu-cam-calibration)

