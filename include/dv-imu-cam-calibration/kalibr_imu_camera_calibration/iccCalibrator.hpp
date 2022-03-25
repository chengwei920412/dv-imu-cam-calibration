//
// Created by radam on 2021-03-25.
//

#pragma once

#include <kalibr_imu_camera_calibration/common.hpp>
#include <kalibr_imu_camera_calibration/iccSensors.hpp>

#include <aslam/backend/BSplineMotionErrorFactory.hpp>
#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/EuclideanDirection.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/SparseCholeskyLinearSystemSolver.hpp>
#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <bsplines/BSplinePose.hpp>

#include <boost/make_shared.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>
#include <iostream>
#include <thread>
#include <vector>

namespace IccCalibratorUtils {
struct ErrorInfo {
    double meanReprojectionError;
    double meanGyroscopeError;
    double meanAccelerometerError;

    ErrorInfo(const double repr, const double gyr, const double acc) :
        meanReprojectionError(repr), meanGyroscopeError(gyr), meanAccelerometerError(acc) {
    }
};

struct CalibrationResult {
    double t_cam_imu;
    Eigen::Matrix4d T_cam_imu;
    bool converged;
    ErrorInfo error_info;

    CalibrationResult(
        const double timeShift,
        const Eigen::Matrix4d& transformation,
        bool conv,
        const ErrorInfo& err_info) :
        t_cam_imu(timeShift),
        T_cam_imu(transformation), converged(conv), error_info(err_info) {
    }
};

static void printResult(const IccCalibratorUtils::CalibrationResult& result, std::stringstream& ss) {
    ss << "Optimization converged:" << std::endl;
    ss << "  " << (result.converged ? "true" : "false") << std::endl;
    ss << "Transformation T_cam_imu:" << std::endl;
    ss << result.T_cam_imu << std::endl;
    ss << "Camera to imu time: [s] (t_imu = t_cam + shift):" << std::endl;
    ss << "  " << result.t_cam_imu << std::endl;
}
} // namespace IccCalibratorUtils

template<typename CameraGeometryType, typename DistortionType>
class IccCalibrator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
    const size_t calibrationGroupId = 0;

    boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseDv = nullptr;
    boost::shared_ptr<aslam::calibration::OptimizationProblem> problem = nullptr;

    boost::shared_ptr<IccImu> iccImu = nullptr;
    boost::shared_ptr<IccCamera<CameraGeometryType, DistortionType>> iccCamera = nullptr;

    boost::shared_ptr<aslam::backend::DesignVariable> gravityDv = nullptr;
    boost::shared_ptr<aslam::backend::EuclideanExpression> gravityExpression = nullptr;

    bool converged = false;
    bool problemBuilt = false;

public:
    IccCalibrator(
        boost::shared_ptr<IccCamera<CameraGeometryType, DistortionType>> camera,
        boost::shared_ptr<IccImu> imu) {
        iccCamera = camera;
        iccImu = imu;
    }

    void initDesignVariables(
        boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
        boost::shared_ptr<bsplines::BSplinePose> poseSpline,
        bool noTimeCalibration,
        bool noChainExtrinsics = true,
        bool estimateGravityLength = false,
        const Eigen::Vector3d& initialGravityEstimate = Eigen::Vector3d(0.0, 9.81, 0.0)) {
        // Initialize the system pose spline (always attached to imu0)
        poseDv = boost::make_shared<aslam::splines::BSplinePoseDesignVariable>(*poseSpline);
        addSplineDesignVariables(problem, poseDv);

        // Add the calibration target orientation design variable. (expressed as gravity vector in target frame)
        if (estimateGravityLength) {
            auto grDv = boost::make_shared<aslam::backend::EuclideanPoint>(initialGravityEstimate);
            gravityExpression = boost::make_shared<aslam::backend::EuclideanExpression>(grDv->toExpression());
            grDv->setActive(true);
            gravityDv = grDv;
        } else {
            auto grDv = boost::make_shared<aslam::backend::EuclideanDirection>(initialGravityEstimate);
            gravityExpression = boost::make_shared<aslam::backend::EuclideanExpression>(grDv->toExpression());
            grDv->setActive(true);
            gravityDv = grDv;
        }
        problem->addDesignVariable(gravityDv, HELPER_GROUP_ID);

        // Add all DVs for all IMUs
        iccImu->addDesignVariables(problem);

        // Add all DVs for the camera chain
        iccCamera->addDesignVariables(problem, noChainExtrinsics, noTimeCalibration);
    }

    void addPoseMotionTerms(
        boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
        const double tv,
        const double rv) {
        const auto wt = 1.0 / tv;
        const auto wr = 1.0 / rv;
        Eigen::Matrix<double, 6, 6> W;
        W.setZero();
        W(0, 0) = wt;
        W(1, 1) = wt;
        W(2, 2) = wt;
        W(3, 3) = wr;
        W(4, 4) = wr;
        W(5, 5) = wr;
        const unsigned int errorOrder = 1;
        throw std::runtime_error("Not implemented addPoseMotionTerms");
        // aslam::backend::addMotionErrorTerms(problem, *poseDv, W, errorOrder);
    }

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
        bool verbose = false) {
        problemBuilt = true;

        if (!problemBuilt) {
            throw std::runtime_error("Problem was not built before calibrate() was called");
        }

        auto bool2string = [](bool val) {
            return (val ? "true" : "false");
        };

        std::cout << "\tSpline order: " << splineOrder << std::endl;
        std::cout << "\tPose knots per second: " << poseKnotsPerSecond << std::endl;
        std::cout << "\tDo pose motion regularization: " << bool2string(doPoseMotionError) << std::endl;
        std::cout << "\t\txddot translation variance: " << mrTranslationVariance << std::endl;
        std::cout << "\t\txddot rotation variance: " << mrRotationVariance << std::endl;
        std::cout << "\tBias knots per second: " << biasKnotsPerSecond << std::endl;
        std::cout << "\tDo bias motion regularization: " << bool2string(doBiasMotionError) << std::endl;
        std::cout << "\tBlake-Zisserman on reprojection errors " << blakeZisserCam << std::endl;
        std::cout << "\tAcceleration Huber width (sigma): " << huberAccel << std::endl;
        std::cout << "\tGyroscope Huber width (sigma): " << huberGyro << std::endl;
        std::cout << "\tDo time calibration: " << bool2string(!noTimeCalibration) << std::endl;
        std::cout << "\tMax iterations: " << maxIterations << std::endl;
        std::cout << "\tTime offset padding: " << timeOffsetPadding << std::endl;

        // ############################################
        // ## initialize camera chain
        // ############################################
        // #estimate the timeshift for all cameras to the main imu
        if (!noTimeCalibration) {
            iccCamera->findTimeshiftCameraImuPrior(iccImu, false);
        }

        assert(iccCamera);
        assert(iccImu);

        // obtain orientation prior between main imu and camera chain (if no external input provided)
        // and initial estimate for the direction of gravity
        iccCamera->findOrientationPriorCameraToImu(iccImu);
        const auto estimatedGravity = iccCamera->getEstimatedGravity();

        // ############################################
        // ## init optimization problem
        // ############################################
        // #initialize a pose spline using the camera poses in the camera chain
        const auto poseSpline = iccCamera->initPoseSplineFromCamera(splineOrder, poseKnotsPerSecond, timeOffsetPadding);

        // Initialize bias splines for all IMUs
        iccImu->initBiasSplines(poseSpline, splineOrder, biasKnotsPerSecond);

        // Now I can build the problem
        problem = boost::make_shared<aslam::calibration::OptimizationProblem>();

        // Initialize all design variables
        initDesignVariables(problem, poseSpline, noTimeCalibration, noChainExtrinsics, false, estimatedGravity);

        // ############################################
        // ## add error terms
        // ############################################
        // #Add calibration target reprojection error terms for all camera in chain
        iccCamera->addCameraErrorTerms(problem, poseDv, blakeZisserCam, timeOffsetPadding);

        // # Initialize IMU error terms.
        iccImu->addAccelerometerErrorTerms(
            problem,
            poseDv,
            gravityExpression->toValue(),
            huberAccel,
            accelNoiseScale = accelNoiseScale);
        iccImu->addGyroscopeErrorTerms(problem, poseDv, gravityExpression->toValue(), huberGyro, gyroNoiseScale);

        // # Add the bias motion terms.
        if (doBiasMotionError) {
            iccImu->addBiasMotionTerms(problem);
        }

        // # Add the pose motion terms.
        if (doPoseMotionError) {
            addPoseMotionTerms(problem, mrTranslationVariance, mrRotationVariance);
        }
    }

    void optimize(
        boost::shared_ptr<aslam::backend::Optimizer2Options> options = nullptr,
        const size_t maxIterations = 30,
        const bool recoverCov = false) {
        if (options == nullptr) {
            options = boost::make_shared<aslam::backend::Optimizer2Options>();
            options->verbose = true;
            const double levenbergMarquardtLambdaInit = 10.0;
            options->nThreads = std::max(1u, std::thread::hardware_concurrency() - 1);
            options->convergenceDeltaX = 1e-5;
            options->convergenceDeltaJ = 1e-2;
            options->maxIterations = maxIterations;
            options->trustRegionPolicy
                = boost::make_shared<aslam::backend::LevenbergMarquardtTrustRegionPolicy>(levenbergMarquardtLambdaInit);
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
            converged = retval.iterations < options->maxIterations;
        } catch (...) {
            optimizationFailed = true;
        }

        if (optimizationFailed) {
            throw std::runtime_error("Optimization failed");
        }

        if (recoverCov) {
            recoverCovariance();
        }
    }

    void recoverCovariance() {
        std::cout << "Recovering covariance..." << std::endl;

        aslam::calibration::IncrementalEstimator estimator(calibrationGroupId);
        auto rval = estimator.addBatch(problem, true);
        auto est_stds = estimator.getSigma2Theta().diagonal().cwiseSqrt();

        throw std::runtime_error("Not implemented recoverCovariance");

        // # split and store the variance
        // self.std_trafo_ic = np.array(est_stds[0:6])
        // self.std_times = np.array(est_stds[6:])
    }

    IccCalibratorUtils::CalibrationResult getResult() {
        IccCalibratorUtils::ErrorInfo error_info(
            iccCamera->getMeanReprojectionError(),
            iccImu->getMeanGyroscopeError(),
            iccImu->getMeanAccelerometerError());

        IccCalibratorUtils::CalibrationResult result(
            iccCamera->getResultTimeShift(),
            iccCamera->getTransformation().T(),
            converged,
            error_info);
        return result;
    }

    void printErrorStatistics(std::stringstream& ss) {
        ss << std::endl << "Normalized Residuals" << std::endl << "-------------------" << std::endl;
        iccCamera->printNormalizedResiduals(ss);
        iccImu->printNormalizedResiduals(ss);

        ss << std::endl << "Residuals" << std::endl << "-------------------" << std::endl;
        iccCamera->printResiduals(ss);
        iccImu->printResiduals(ss);
    }
};
