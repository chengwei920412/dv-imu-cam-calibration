//
// Created by radam on 2021-03-25.
//

#include <kalibr_imu_camera_calibration/common.hpp>
#include <kalibr_imu_camera_calibration/iccCalibrator.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/backend/EuclideanDirection.hpp>
#include <aslam/backend/EuclideanPoint.hpp>


#include <iostream>


void addSplineDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
							  boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> dvc,
							  bool setActive=true,
							  size_t groupId = HELPER_GROUP_ID) {
  for (size_t i = 0 ; i < dvc->numDesignVariables() ; ++i) {
    auto dv = dvc->designVariable(i);
    dv->setActive(setActive);
    //problem->addDesignVariable(dv, groupId); // TODO(radam): fix type
  }

}

IccCalibrator::IccCalibrator() {

}



// TODO(radam): use boost shared pointer
void IccCalibrator::initDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
										boost::shared_ptr<bsplines::BSplinePose> poseSpline,
										bool noTimeCalibration,
										bool noChainExtrinsics,
										bool estimateGravityLength,
										const Eigen::Vector3d &initialGravityEstimate) {
  // Initialize the system pose spline (always attached to imu0)
  poseDv = boost::make_shared<aslam::splines::BSplinePoseDesignVariable>(*poseSpline);
  addSplineDesignVariables(problem, poseDv);

  // Add the calibration target orientation design variable. (expressed as gravity vector in target frame)
  if (estimateGravityLength) {
    auto grDv = boost::make_shared<aslam::backend::EuclideanPoint>(initialGravityEstimate);
	gravityExpression = boost::make_shared<aslam::backend::EuclideanExpression>(grDv->toExpression());
	grDv->setActive(true);
    gravityDv = grDv;
  } else {
    auto grDv = boost::make_shared<aslam::backend::EuclideanDirection>(initialGravityEstimate);
	gravityExpression = boost::make_shared<aslam::backend::EuclideanExpression>(grDv->toExpression());
	grDv->setActive(true);
    gravityDv = grDv;
  }
  problem->addDesignVariable(gravityDv, HELPER_GROUP_ID);

  // Add all DVs for all IMUs
  iccImu->addDesignVariables(problem);

  // Add all DVs for the camera chain
  iccCamera->addDesignVariables(problem, noTimeCalibration, noChainExtrinsics);
}

void IccCalibrator::addPoseMotionTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
									   const double tv,
									   const double rv) {
  const auto wt = 1.0/tv;
  const auto wr = 1.0/rv;
  Eigen::Matrix<double, 6,6> W;
  W.setZero();
  W(0,0) = wt;
  W(1,1) = wt;
  W(2,2) = wt;
  W(3,3) = wr;
  W(4,4) = wr;
  W(5,5) = wr;
  // TODO(radam): order is incorrect
  const unsigned int errorOrder = 1;
  throw std::runtime_error("Not implemented addPoseMotionTerms");
  //aslam::backend::addMotionErrorTerms(problem, *poseDv, W, errorOrder);
}

void IccCalibrator::registerCamera(boost::shared_ptr<IccCamera> camera) {
  iccCamera = camera;
}

void IccCalibrator::registerImu(boost::shared_ptr<IccImu> imu) {
  iccImu = imu;
}


void IccCalibrator::optimize(boost::shared_ptr<aslam::backend::Optimizer2Options> options,
							 const size_t maxIterations,
							 const bool recoverCov) {

  if (options == nullptr) {
	options = boost::make_shared<aslam::backend::Optimizer2Options>();
	options->verbose = true;
	// options.doLevenbergMarquardt = True // TODO(radam): ??
	const double levenbergMarquardtLambdaInit = 10.0;
	options->nThreads = std::max(1u, std::thread::hardware_concurrency()-1);
	options->convergenceDeltaX = 1e-5;
	options->convergenceDeltaJ = 1e-2;
	options->maxIterations = maxIterations;
	options->trustRegionPolicy = boost::make_shared<aslam::backend::LevenbergMarquardtTrustRegionPolicy>(levenbergMarquardtLambdaInit);
	options->linearSystemSolver = boost::make_shared<aslam::backend::BlockCholeskyLinearSystemSolver>();

  }

  auto optimizer = aslam::backend::Optimizer2(*options);
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

}

void IccCalibrator::recoverCovariance() {
  std::cout << "Recovering covariance..." << std::endl;

  aslam::calibration::IncrementalEstimator estimator(calibrationGroupId);
  auto rval = estimator.addBatch(problem, true);
  auto est_stds = estimator.getSigma2Theta().diagonal().cwiseSqrt();

  // TODO(radam): finish once knowing what the dimensions are

  // # split and store the variance
  // self.std_trafo_ic = np.array(est_stds[0:6])
  // self.std_times = np.array(est_stds[6:])
}