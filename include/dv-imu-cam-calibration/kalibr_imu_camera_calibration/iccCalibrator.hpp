//
// Created by radam on 2021-03-25.
//

#pragma once

#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp>

#include <bsplines/BSplinePose.hpp>

#include <boost/make_shared.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>

#include <vector>
#include <thread>

class iccCalibrator {

public:

protected:

  const size_t calibrationGroupId = 0;

  std::unique_ptr<aslam::splines::BSplinePoseDesignVariable> poseDv = nullptr;
  boost::shared_ptr<aslam::calibration::OptimizationProblem> problem = nullptr;



public:

  iccCalibrator();

  // TODO(radam): move to cpp
  void initDesignVariables(std::unique_ptr<aslam::calibration::OptimizationProblem>& problem,
						   const bsplines::BSplinePose& poseSpline,
						   bool noTimeCalibration,
						   bool noChainExtrinsics=true,
						   bool estimateGravityLength=false,
						   const Eigen::Vector3d& initialGravityEstimate=Eigen::Vector3d(0.0, 9.81, 0.0));




  void addPoseMotionTerms(){} // TODO(radam):

  void registerCamChain() {} // TODO(radam):

  void registerImu() {} // TODO(radam):

  void buildProblem(int splineOrder=6,
					int poseKnotsPerSecond=70,
					int biasKnotsPerSecond = 70,
					bool doPoseMotionError=false,
					double mrTranslationVariance=1e6,
					double mrRotationVariance=1e5,
					bool doBiasMotionError=true,
					int blakeZisserCam=-1,
					int huberAccel=-1,
					int huberGyro=-1,
					bool noTimeCalibration=false,
					bool noChainExtrinsics=true,
					int maxIterations=20,
					double gyroNoiseScale=1.0,
					double accelNoiseScale=1.0,
					double timeOffsetPadding=0.02,
					bool verbose=false
  ) {

    if (!noTimeCalibration) {
      throw std::runtime_error("Time shift calibration is not implemented.");
    }

    // TODO(radam): implement the rest

  } // TODO(radam): move to cpp


  void optimize(const bool opts, // TODO(radam): check if options are passed
				const size_t maxIterations = 30,
				const bool recoverCov = false) {
    aslam::backend::Optimizer2Options options;
    options.verbose = true;
	// options.doLevenbergMarquardt = True // TODO(radam): ??
	const double levenbergMarquardtLambdaInit = 10.0;
	options.nThreads = std::max(1u, std::thread::hardware_concurrency()-1);
	options.convergenceDeltaX = 1e-5;
	options.convergenceDeltaJ = 1e-2;
	options.maxIterations = maxIterations;
	options.trustRegionPolicy = boost::make_shared<aslam::backend::LevenbergMarquardtTrustRegionPolicy>(levenbergMarquardtLambdaInit);
	options.linearSystemSolver = boost::make_shared<aslam::backend::BlockCholeskyLinearSystemSolver>();

	auto optimizer = aslam::backend::Optimizer2(options);
	optimizer.setProblem(problem);

	bool optimizationFailed = false;
	try {
	  const auto retval = optimizer.optimize();
	  if (retval.linearSolverFailure) {
	    optimizationFailed = true;
	  }
	} catch(...) {
	  optimizationFailed = true;
	}

	if (optimizationFailed) {
	  throw std::runtime_error("Optimization failed");
	}

	if (recoverCov) {
	  recoverCovariance();
	}


  } // TODO(radam): move to cpp

  void recoverCovariance();




};
