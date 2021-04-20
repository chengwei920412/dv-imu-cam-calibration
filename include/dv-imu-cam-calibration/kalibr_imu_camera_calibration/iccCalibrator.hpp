//
// Created by radam on 2021-03-25.
//

#pragma once

#include <kalibr_imu_camera_calibration/iccSensors.hpp>

#include <aslam/backend/BSplineMotionErrorFactory.hpp>
#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/SparseCholeskyLinearSystemSolver.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <bsplines/BSplinePose.hpp>

#include <boost/make_shared.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>
#include <thread>
#include <vector>

class IccCalibrator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct CalibrationResult {
        double t_cam_imu;
        Eigen::Matrix4d T_cam_imu;
        bool converged;

        CalibrationResult(const double timeShift, const Eigen::Matrix4d& transformation, bool conv) :
            t_cam_imu(timeShift), T_cam_imu(transformation), converged(conv) {
        }
    };

protected:
    const size_t calibrationGroupId = 0;

    boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseDv = nullptr;
    boost::shared_ptr<aslam::calibration::OptimizationProblem> problem = nullptr;

    boost::shared_ptr<IccImu> iccImu = nullptr;
    boost::shared_ptr<IccCamera> iccCamera = nullptr;

    boost::shared_ptr<aslam::backend::DesignVariable> gravityDv = nullptr;
    boost::shared_ptr<aslam::backend::EuclideanExpression> gravityExpression = nullptr;

    bool converged = false;
    bool problemBuilt = false;

public:
    IccCalibrator(boost::shared_ptr<IccCamera> camera, boost::shared_ptr<IccImu> imu);

    void initDesignVariables(
        boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
        boost::shared_ptr<bsplines::BSplinePose> poseSpline,
        bool noTimeCalibration,
        bool noChainExtrinsics = true,
        bool estimateGravityLength = false,
        const Eigen::Vector3d& initialGravityEstimate = Eigen::Vector3d(0.0, 9.81, 0.0));

    void addPoseMotionTerms(
        boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
        const double tv,
        const double rv);

    void buildProblem(
        size_t splineOrder = 6,
        size_t poseKnotsPerSecond = 70,
        size_t biasKnotsPerSecond = 70,
        bool doPoseMotionError = false,
        double mrTranslationVariance = 1e6,
        double mrRotationVariance = 1e5,
        bool doBiasMotionError = true,
        int blakeZisserCam = -1,
        int huberAccel = -1,
        int huberGyro = -1,
        bool noTimeCalibration = false,
        bool noChainExtrinsics = true,
        int maxIterations = 20,
        double gyroNoiseScale = 1.0,
        double accelNoiseScale = 1.0,
        double timeOffsetPadding = 0.02,
        bool verbose = false);

    void optimize(
        boost::shared_ptr<aslam::backend::Optimizer2Options> options = nullptr,
        const size_t maxIterations = 30,
        const bool recoverCov = false);

    void recoverCovariance();

    CalibrationResult getResult();

    static void printResult(const CalibrationResult& result, std::stringstream& ss);

    void printErrorStatistics(std::stringstream& ss);
};
