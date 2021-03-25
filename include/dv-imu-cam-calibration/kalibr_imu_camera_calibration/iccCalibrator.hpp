//
// Created by radam on 2021-03-25.
//

#pragma once

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>

#include <vector>

class iccCalibrator {

public:

protected:

  const size_t calibrationGroupId = 0;

public:

  iccCalibrator();

  // TODO(radam): move to cpp
  void initDesignVariables(bool problem, bool poseSpline, bool noTimeCalibration, bool noChainExtrinsics=true,
						   bool estimateGravityLength=false, const Eigen::Vector3d& initialGravityEstimate=Eigen::Vector3d(0.0, 9.81, 0.0)) {
    //auto poseDv =

  }


  void recoverCovariance();



};
