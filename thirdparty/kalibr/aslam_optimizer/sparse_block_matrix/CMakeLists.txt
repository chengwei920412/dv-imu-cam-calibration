cmake_minimum_required(VERSION 2.8.3)

project(sparse_block_matrix)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(SuiteSparse REQUIRED)
include_directories(SYSTEM ${SUITESPARSE_INCLUDE_DIRS})

add_definitions(-fPIC)

add_library(${PROJECT_NAME} SHARED
  src/matrix_structure.cpp
  src/sparse_helper.cpp
  src/marginal_covariance_cholesky.cpp
)

target_link_libraries(${PROJECT_NAME}
        sm_common
        sm_eigen
        ${SUITESPARSE_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${SUITESPARSE_INCLUDE_DIRS}
        )

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${DV_MODULES_DIR}/imu-calibration)
