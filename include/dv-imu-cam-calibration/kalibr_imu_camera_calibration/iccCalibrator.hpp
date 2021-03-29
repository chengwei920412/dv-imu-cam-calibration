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

class IccCalibrator {

public:

protected:

  const size_t calibrationGroupId = 0;

  boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseDv = nullptr;
  boost::shared_ptr<aslam::calibration::OptimizationProblem> problem = nullptr;



public:

  IccCalibrator();

  // TODO(radam): move to cpp
  void initDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
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


    // Now I can build the problem
    problem = boost::make_shared<aslam::calibration::OptimizationProblem>();

    // Initialize all design variables
    //initDesignVariables(problem, ) // TODO(radam): finish

  } // TODO(radam): move to cpp


  void optimize(boost::shared_ptr<aslam::backend::Optimizer2Options> options = nullptr,
				const size_t maxIterations = 30,
				const bool recoverCov = false) ;

  void recoverCovariance();




};
