//
// Created by radam on 2021-03-25.
//

#pragma once

#include <kalibr_imu_camera_calibration/iccSensors.hpp>

#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/BSplineMotionErrorFactory.hpp>
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

  boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseDv = nullptr; // TODO(radam): maybe base class?
  boost::shared_ptr<aslam::calibration::OptimizationProblem> problem = nullptr;


  boost::shared_ptr<IccImu> iccImu = nullptr;
  boost::shared_ptr<IccCamera> iccCamera = nullptr;

  boost::shared_ptr<aslam::backend::DesignVariable> gravityDv = nullptr;
  boost::shared_ptr<aslam::backend::EuclideanExpression> gravityExpression = nullptr;


public:

  IccCalibrator();

  // TODO(radam): move to cpp
  void initDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
						   boost::shared_ptr<bsplines::BSplinePose> poseSpline,
						   bool noTimeCalibration,
						   bool noChainExtrinsics=true,
						   bool estimateGravityLength=false,
						   const Eigen::Vector3d& initialGravityEstimate=Eigen::Vector3d(0.0, 9.81, 0.0));




  void addPoseMotionTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
						  const double tv,
						  const double rv);





  void registerCamera(boost::shared_ptr<IccCamera> camera);

  void registerImu(boost::shared_ptr<IccImu> imu);



  void buildProblem(size_t splineOrder=6,
					size_t poseKnotsPerSecond=70,
					size_t biasKnotsPerSecond = 70,
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

	// ############################################
	// ## initialize camera chain
	// ############################################
    // #estimate the timeshift for all cameras to the main imu
    if (!noTimeCalibration) {
      throw std::runtime_error("Time shift calibration is not implemented.");
    }

    assert(iccCamera);
    assert(iccImu);

    // obtain orientation prior between main imu and camera chain (if no external input provided)
	// and initial estimate for the direction of gravity
    iccCamera->findOrientationPriorCameraToImu(iccImu);
    const auto estimatedGravity = iccCamera->getEstimatedGravity();


	// ############################################
	// ## init optimization problem
	// ############################################
	// #initialize a pose spline using the camera poses in the camera chain
	const auto poseSpline = iccCamera->initPoseSplineFromCamera(splineOrder, poseKnotsPerSecond, timeOffsetPadding);

    // Initialize bias splines for all IMUs
	iccImu->initBiasSplines(poseSpline, splineOrder, biasKnotsPerSecond);

	// Now I can build the problem
    problem = boost::make_shared<aslam::calibration::OptimizationProblem>();

    // Initialize all design variables
    initDesignVariables(problem, poseSpline, noTimeCalibration, noChainExtrinsics, false, estimatedGravity);

	// ############################################
	// ## add error terms
	// ############################################
    // #Add calibration target reprojection error terms for all camera in chain
    iccCamera->addCameraErrorTerms(problem, poseDv, blakeZisserCam, timeOffsetPadding);

    // # Initialize IMU error terms.
	iccImu->addAccelerometerErrorTerms(problem, poseDv, gravityExpression->toValue(), huberAccel, accelNoiseScale=accelNoiseScale);
	iccImu->addGyroscopeErrorTerms(problem, poseDv, gravityExpression->toValue(), huberGyro, gyroNoiseScale);

    // # Add the bias motion terms.
	if (doBiasMotionError) {
	  iccImu->addBiasMotionTerms(problem);
	}

    // # Add the pose motion terms.
	if (doPoseMotionError) {
	  addPoseMotionTerms(problem, mrTranslationVariance, mrRotationVariance);
	}

  } // TODO(radam): move to cpp


  void optimize(boost::shared_ptr<aslam::backend::Optimizer2Options> options = nullptr,
				const size_t maxIterations = 30,
				const bool recoverCov = false) ;

  void recoverCovariance();




};
