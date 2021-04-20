//
// Created by radam on 2021-04-12.
//

#include <kalibr_imu_camera_calibration/common.hpp>

#include <vector>

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

#include <numeric>

std::tuple<double, double, double> errorStatistics(std::vector<double> vals) {
    std::tuple<double, double, double> result = std::make_tuple(-1.0, -1.0, -1.0);
    if (vals.size() < 2) {
        return result;
    }

    const double mean = std::accumulate(vals.begin(), vals.end(), 0.0) / static_cast<double>(vals.size());
    std::get<0>(result) = mean;

    double median;
    std::sort(vals.begin(), vals.end());
    if (vals.size() % 2 == 0) {
        median = (vals.at(vals.size() / 2 - 1) + vals.at(vals.size() / 2)) / 2.;
    } else {
        median = vals.at(vals.size() / 2);
    }
    std::get<1>(result) = median;

    double std = 0.0;
    for (const auto& v : vals) {
        std += std::pow(v - mean, 2.0);
    }
    std = std::sqrt(std / static_cast<double>(vals.size()));

    std::get<2>(result) = std;

    return result;
}