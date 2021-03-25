cmake_minimum_required(VERSION 2.8.3)
project(incremental_calibration)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/../cmake/)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

if(APPLE)
  set(CMAKE_CXX_FLAGS "-std=c++11")
else()
  set(CMAKE_CXX_FLAGS "-std=c++0x")
endif()

add_library(${PROJECT_NAME}
  src/base/Serializable.cpp
  src/base/Timestamp.cpp
  src/base/Condition.cpp
  src/base/Mutex.cpp
  src/base/Thread.cpp
  src/base/Threads.cpp
  src/base/Timer.cpp
  src/exceptions/Exception.cpp
  src/exceptions/InvalidOperationException.cpp
  src/exceptions/NullPointerException.cpp
  src/exceptions/SystemException.cpp
  src/statistics/NormalDistribution1v.cpp
  src/statistics/ChiSquareDistribution.cpp
  src/statistics/EstimatorMLNormal1v.cpp
  src/functions/IncompleteGammaPFunction.cpp
  src/functions/IncompleteGammaQFunction.cpp
  src/functions/LogFactorialFunction.cpp
  src/functions/LogGammaFunction.cpp
  src/core/IncrementalEstimator.cpp
  src/core/OptimizationProblem.cpp
  src/core/IncrementalOptimizationProblem.cpp
  src/core/LinearSolver.cpp
  src/core/LinearSolverOptions.cpp
  src/algorithms/marginalize.cpp
  src/algorithms/linalg.cpp
)

find_package(Boost REQUIRED COMPONENTS system thread)
target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${TBB_LIBRARIES}
        aslam_backend
        aslam_backend_expressions)
target_include_directories(${PROJECT_NAME} PUBLIC
        include)
