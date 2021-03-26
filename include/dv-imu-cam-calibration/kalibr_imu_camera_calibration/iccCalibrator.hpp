//
// Created by radam on 2021-03-25.
//

#pragma once

#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>

#include <bsplines/BSplinePose.hpp>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>

#include <vector>

class iccCalibrator {

public:

protected:

  const size_t calibrationGroupId = 0;

  std::unique_ptr<aslam::splines::BSplinePoseDesignVariable> poseDv = nullptr;
  std::unique_ptr<aslam::calibration::OptimizationProblem> problem = nullptr;



public:

  iccCalibrator();

  // TODO(radam): move to cpp
  void initDesignVariables(std::unique_ptr<aslam::calibration::OptimizationProblem>& problem,
						   const bsplines::BSplinePose& poseSpline,
						   bool noTimeCalibration,
						   bool noChainExtrinsics=true,
						   bool estimateGravityLength=false,
						   const Eigen::Vector3d& initialGravityEstimate=Eigen::Vector3d(0.0, 9.81, 0.0));


  void recoverCovariance();

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

  }



};
