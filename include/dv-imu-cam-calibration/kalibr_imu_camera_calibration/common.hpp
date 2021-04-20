//
// Created by radam on 2021-03-29.
//

#pragma once

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>

#include <boost/make_shared.hpp>

#include <cstddef>

static constexpr size_t CALIBRATION_GROUP_ID = 0;
static constexpr size_t HELPER_GROUP_ID = 1;

void addSplineDesignVariables(
    boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
    boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> dvc,
    bool setActive = true,
    size_t groupId = HELPER_GROUP_ID);

void addSplineDesignVariables(
    boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
    boost::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> dvc,
    bool setActive = true,
    size_t groupId = HELPER_GROUP_ID);

std::tuple<double, double, double> errorStatistics(std::vector<double> vals);
