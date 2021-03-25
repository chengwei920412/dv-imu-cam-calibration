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