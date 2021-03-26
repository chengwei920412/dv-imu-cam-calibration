//
// Created by radam on 2021-03-25.
//

#include <kalibr_imu_camera_calibration/iccCalibrator.hpp>



#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/backend/EuclideanDirection.hpp>
#include <aslam/backend/EuclideanPoint.hpp>


static constexpr size_t HELPER_GROUP_ID = 1;

void addSplineDesignVariables(std::unique_ptr<aslam::calibration::OptimizationProblem>& problem,
							  std::unique_ptr<aslam::splines::BSplinePoseDesignVariable>& dvc,
							  bool setActive=true,
							  size_t groupId = HELPER_GROUP_ID) {
  for (size_t i = 0 ; i < dvc->numDesignVariables() ; ++i) {
    auto dv = dvc->designVariable(i);
    dv->setActive(setActive);
    //problem->addDesignVariable(dv, groupId); // TODO(radam): fix type
  }

}

iccCalibrator::iccCalibrator() {

}


void iccCalibrator::recoverCovariance() {
  std::cout << "Recovering covariance..." << std::endl;

  aslam::calibration::IncrementalEstimator estimator(calibrationGroupId);
  //auto rval = estimator.addBatch(problem, true);
  // TODO(radam): finish


}

void iccCalibrator::initDesignVariables(std::unique_ptr<aslam::calibration::OptimizationProblem>& problem,
										const bsplines::BSplinePose& poseSpline,
										bool noTimeCalibration,
										bool noChainExtrinsics,
										bool estimateGravityLength,
										const Eigen::Vector3d &initialGravityEstimate) {
  // Initialize the system pose spline (always attached to imu0)
  poseDv = std::make_unique<aslam::splines::BSplinePoseDesignVariable>(poseSpline);
  addSplineDesignVariables(problem, poseDv);

  // Add the calibration target orientation design variable. (expressed as gravity vector in target frame)
  if (estimateGravityLength) {
    auto gravityDv = aslam::backend::EuclideanPoint(initialGravityEstimate); // TODO(radam): not local
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