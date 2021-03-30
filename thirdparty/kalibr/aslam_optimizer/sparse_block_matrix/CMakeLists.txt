cmake_minimum_required(VERSION 2.8.3)

project(sparse_block_matrix)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_definitions(-fPIC)

add_library(${PROJECT_NAME}
  src/matrix_structure.cpp
  src/sparse_helper.cpp
  src/marginal_covariance_cholesky.cpp
)

target_link_libraries(${PROJECT_NAME}
        sm_common
        sm_eigen
        suitesparse)

target_include_directories(${PROJECT_NAME} PUBLIC
        include)