//
// Created by radam on 2021-03-25.
//

#include <kalibr_imu_camera_calibration/iccCalibrator.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/backend/EuclideanDirection.hpp>
#include <aslam/backend/EuclideanPoint.hpp>


static constexpr size_t HELPER_GROUP_ID = 1;

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
										const bsplines::BSplinePose& poseSpline,
										bool noTimeCalibration,
										bool noChainExtrinsics,
										bool estimateGravityLength,
										const Eigen::Vector3d &initialGravityEstimate) {
  // Initialize the system pose spline (always attached to imu0)
  poseDv = boost::make_shared<aslam::splines::BSplinePoseDesignVariable>(poseSpline);
  addSplineDesignVariables(problem, poseDv);

  // Add the calibration target orientation design variable. (expressed as gravity vector in target frame)
  if (estimateGravityLength) {
    auto gravityDv = aslam::backend::EuclideanPoint(initialGravityEstimate); // TODO(radam): not local
    gravityDv.toExpression();
  } else {
    auto gravityDv = aslam::backend::EuclideanDirection(initialGravityEstimate); // TODO(radam): not local
  }
  // gravityExpression = gravityDv.toExpression(); // TODO(radam): fix
  // gravityDv.setActive(true); // TODO(radam): fix

//  self.gravityExpression = self.gravityDv.toExpression()
//  self.gravityDv.setActive( True )
//  problem.addDesignVariable(self.gravityDv, HELPER_GROUP_ID)

  // TODO(radam): add design variables from IMU

  // TODO(radam): add design variables to camera chain

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