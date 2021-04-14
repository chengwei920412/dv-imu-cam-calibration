//
// Created by radam on 2021-04-12.
//

#include <kalibr_imu_camera_calibration/common.hpp>


void addSplineDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
							  boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> dvc,
							  bool setActive,
							  size_t groupId) {
  for (size_t i = 0 ; i < dvc->numDesignVariables() ; ++i) {
	auto dv = dvc->designVariable(i);
	dv->setActive(setActive);
	auto boostDv = boost::shared_ptr<aslam::backend::DesignVariable>(dv);
	problem->addDesignVariable(boostDv, groupId);
  }
}

void addSplineDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
							  boost::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> dvc,
							  bool setActive,
							  size_t groupId) {
  for (size_t i = 0 ; i < dvc->numDesignVariables() ; ++i) {
	auto dv = dvc->designVariable(i);
	dv->setActive(setActive);
	auto boostDv = boost::shared_ptr<aslam::backend::DesignVariable>(dv);
	problem->addDesignVariable(boostDv, groupId);
  }
}