cmake_minimum_required(VERSION 2.8.3)
project(aslam_splines)

find_package(Boost REQUIRED COMPONENTS system )

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_definitions(-fPIC)

add_library(${PROJECT_NAME}
  src/BSplinePoseDesignVariable.cpp
  src/BSplineExpressions.cpp
  src/EuclideanBSplineDesignVariable.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        aslam_backend
        aslam_backend_expressions
        bsplines)

target_include_directories(${PROJECT_NAME} PUBLIC
        include)
