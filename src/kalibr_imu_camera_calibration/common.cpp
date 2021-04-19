//
// Created by radam on 2021-04-12.
//

#include <kalibr_imu_camera_calibration/common.hpp>

/**
 * Dummy dealocation function used to avoid double destruction of shared pointer.
 * @tparam T
 * @param object
 */
template<class T>
static void Deallocate(T* object) {
}

void addSplineDesignVariables(
    boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
    boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> dvc,
    bool setActive,
    size_t groupId) {
    for (size_t i = 0; i < dvc->numDesignVariables(); ++i) {
        auto dv = dvc->designVariable(i);
        dv->setActive(setActive);
        auto boostDv
            = boost::shared_ptr<aslam::backend::DesignVariable>(dv, &Deallocate<aslam::backend::DesignVariable>);
        problem->addDesignVariable(boostDv, groupId);
    }
}

void addSplineDesignVariables(
    boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
    boost::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> dvc,
    bool setActive,
    size_t groupId) {
    for (size_t i = 0; i < dvc->numDesignVariables(); ++i) {
        auto dv = dvc->designVariable(i);
        dv->setActive(setActive);
        auto boostDv
            = boost::shared_ptr<aslam::backend::DesignVariable>(dv, &Deallocate<aslam::backend::DesignVariable>);
        problem->addDesignVariable(boostDv, groupId);
    }
}