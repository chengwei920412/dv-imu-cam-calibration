cmake_minimum_required(VERSION 2.8.3)
project(aslam_backend)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_definitions(-fPIC)

add_library(${PROJECT_NAME} SHARED
  src/MEstimatorPolicies.cpp
  src/JacobianContainer.cpp
  src/DesignVariable.cpp
  src/ErrorTerm.cpp
  src/OptimizationProblemBase.cpp
  src/Optimizer.cpp
  src/sparse_matrix_functions.cpp
  src/OptimizationProblem.cpp
  src/SimpleOptimizationProblem.cpp
  src/JacobianBuilder.cpp
  src/LinearSystemSolver.cpp
  src/BlockCholeskyLinearSystemSolver.cpp
  src/SparseCholeskyLinearSystemSolver.cpp
  src/SparseQrLinearSystemSolver.cpp
  src/Matrix.cpp
  src/DenseMatrix.cpp
  src/SparseBlockMatrixWrapper.cpp
  src/DenseQrLinearSystemSolver.cpp
  src/BlockCholeskyLinearSolverOptions.cpp
  src/SparseCholeskyLinearSolverOptions.cpp
  src/SparseQRLinearSolverOptions.cpp
  src/DenseQRLinearSolverOptions.cpp
  src/TrustRegionPolicy.cpp
  src/ErrorTermDs.cpp
  src/GaussNewtonTrustRegionPolicy.cpp
  src/LevenbergMarquardtTrustRegionPolicy.cpp
  src/Marginalizer.cpp
  src/MarginalizationPriorErrorTerm.cpp
  src/DogLegTrustRegionPolicy.cpp
  src/Optimizer2.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${TBB_LIBRARIES}
        sparse_block_matrix
        sm_boost
        sm_eigen
        sm_random
        sm_timing
        sm_logging
        sm_property_tree
        ${SUITESPARSE_LIBRARIES}
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        include)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${DV_MODULES_DIR}/imu-calibration)
