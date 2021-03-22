cmake_minimum_required(VERSION 2.8.3)

project(sparse_block_matrix)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME}
  src/matrix_structure.cpp
  src/sparse_helper.cpp
  src/marginal_covariance_cholesky.cpp
)

