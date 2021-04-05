cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_backend)

find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME}
  src/GridCalibrationTargetDesignVariableContainer.cpp
  # src/CameraGeometryDesignVariableContainer.cpp
  # src/NCameraSystemDesignVariableContainer.cpp
)
target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        aslam_cameras
        aslam_backend_expressions
        )
target_include_directories(${PROJECT_NAME} PUBLIC
        include)