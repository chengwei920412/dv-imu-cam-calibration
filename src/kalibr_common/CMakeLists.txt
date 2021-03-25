
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

add_library(config_reader
        ConfigReader.cpp)
target_link_libraries(config_reader
        aslam_cv)
target_include_directories(config_reader PUBLIC
        ../../include/dv-imu-cam-calibration)

