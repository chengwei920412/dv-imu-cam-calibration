//
// Created by radam on 2021-03-25.
//

#include <kalibr_imu_camera_calibration/iccCalibrator.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>


iccCalibrator::iccCalibrator() {

}


void iccCalibrator::recoverCovariance() {
  std::cout << "Recovering covariance..." << std::endl;

  aslam::calibration::IncrementalEstimator estimator(calibrationGroupId);
  //auto rval = estimator.addBatch(problem, true);
  // TODO(radam): finish


}

void iccCalibrator::initDesignVariables(bool problem,
										const bsplines::BSplinePose& poseSpline,
										bool noTimeCalibration,
										bool noChainExtrinsics,
										bool estimateGravityLength,
										const Eigen::Vector3d &initialGravityEstimate) {
  poseDv = std::make_unique<aslam::splines::BSplinePoseDesignVariable>(poseSpline);



}