#pragma once

#include <kalibr_imu_camera_calibration/common.hpp>

#include <kalibr_common/ConfigReader.hpp>

#include <aslam/Keypoint.hpp>
#include <aslam/backend/BSplineMotionError.hpp>
#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/ReprojectionError.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/SimpleReprojectionError.hpp>
#include <aslam/backend/TransformationBasic.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <bsplines/BSpline.hpp>
#include <kalibr_errorterms/EuclideanError.hpp>
#include <kalibr_errorterms/GyroscopeError.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/transformations.hpp>

#include <boost/make_shared.hpp>

#include <Eigen/Eigen>
#include <iostream>
#include <string>

namespace IccImuUtils {
struct ImuMeasurement {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d omega, alpha;
    Eigen::Matrix3d omegaR, omegaInvR, alphaR, alphaInvR;
    double stamp;

    explicit ImuMeasurement(
        const double _stamp,
        const Eigen::Vector3d& _omega,
        const Eigen::Vector3d& _alpha,
        const Eigen::Matrix3d& Rgyro,
        const Eigen::Matrix3d& Raccel) {
        omega = _omega;
        alpha = _alpha;
        omegaR = Rgyro;
        omegaInvR = Rgyro.inverse();
        alphaR = Raccel;
        alphaInvR = Raccel.inverse();
        stamp = _stamp;
    }
};

} // namespace IccImuUtils

class IccImu {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    Eigen::Vector3d gyroBiasPrior;
    size_t gyroBiasPriorCount = 0;

protected:
    ImuParameters imuParameters;
    double accelUncertaintyDiscrete;
    double accelRandomWalk;
    double accelUncertainty;
    double gyroUncertaintyDiscrete;
    double gyroRandomWalk;
    double gyroUncertainty;

    Eigen::Vector4d q_i_b_prior;
    boost::shared_ptr<aslam::backend::RotationQuaternion> q_i_b_Dv = nullptr;
    boost::shared_ptr<aslam::backend::EuclideanPoint> r_b_Dv = nullptr;

    double timeOffset = 0.0;

    boost::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> gyroBiasDv = nullptr;
    boost::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> accelBiasDv = nullptr;

    // Error terms
    std::vector<boost::shared_ptr<kalibr_errorterms::EuclideanError>> accelErrors;
    std::vector<boost::shared_ptr<kalibr_errorterms::EuclideanError>> gyroErrors;

    // Bias BSplines
    boost::shared_ptr<bsplines::BSpline> gyroBias = nullptr;
    boost::shared_ptr<bsplines::BSpline> accelBias = nullptr;

    // Imu measurement data
    boost::shared_ptr<std::vector<IccImuUtils::ImuMeasurement>> imuData = nullptr;

public:
    double getAccelUncertaintyDiscrete() const {
        return accelUncertaintyDiscrete;
    }

    double getGyroUncertaintyDiscrete() const {
        return gyroUncertaintyDiscrete;
    }

    IccImu(const ImuParameters& imuParams, boost::shared_ptr<std::vector<IccImuUtils::ImuMeasurement>> data) :
        imuParameters(imuParams) {
        imuData = data;

        const auto [aud, arw, au] = imuParameters.getAccelerometerStatistics();
        const auto [gud, grw, gu] = imuParameters.getGyroStatistics();

        accelUncertaintyDiscrete = aud;
        accelRandomWalk = arw;
        accelUncertainty = au;
        gyroUncertaintyDiscrete = gud;
        gyroRandomWalk = grw;
        gyroUncertainty = gu;

        gyroBiasPrior.setZero();

        q_i_b_prior = Eigen::Vector4d(0., 0., 0., 1.);

        std::cout << "Initializing IMU:" << std::endl;
        std::cout << "  Update rate: " << imuParameters.updateRate << std::endl;
        std::cout << "  Accelerometer: " << std::endl;
        std::cout << "    Noise density: " << accelUncertainty << std::endl;
        std::cout << "    Noise density (discrete): " << accelUncertaintyDiscrete << std::endl;
        std::cout << "    Random walk: " << accelRandomWalk << std::endl;
        std::cout << "  Gyroscope: " << std::endl;
        std::cout << "    Noise density: " << gyroUncertainty << std::endl;
        std::cout << "    Noise density (discrete): " << gyroUncertaintyDiscrete << std::endl;
        std::cout << "    Random walk: " << gyroRandomWalk << std::endl;
    }

    std::vector<IccImuUtils::ImuMeasurement>& getImuData() {
        return *imuData;
    }

    void addDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem) {
        gyroBiasDv = boost::make_shared<aslam::splines::EuclideanBSplineDesignVariable>(*gyroBias);
        accelBiasDv = boost::make_shared<aslam::splines::EuclideanBSplineDesignVariable>(*accelBias);

        addSplineDesignVariables(problem, gyroBiasDv, true, HELPER_GROUP_ID);
        addSplineDesignVariables(problem, accelBiasDv, true, HELPER_GROUP_ID);

        q_i_b_Dv = boost::make_shared<aslam::backend::RotationQuaternion>(q_i_b_prior);
        problem->addDesignVariable(q_i_b_Dv, HELPER_GROUP_ID);
        q_i_b_Dv->setActive(false);
        r_b_Dv = boost::make_shared<aslam::backend::EuclideanPoint>(Eigen::Vector3d(0.0, 0.0, 0.0));
        problem->addDesignVariable(r_b_Dv, HELPER_GROUP_ID);
        r_b_Dv->setActive(false);
    }

    void addAccelerometerErrorTerms(
        boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
        boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseSplineDv,
        const Eigen::Vector3d& g_w,
        double mSigma = 0.0,
        double accelNoiseScale = 1.0) {
        std::cout << std::endl << "Adding accelerometer error terms (" << imuData->size() << ") ..." << std::endl;

        const double weight = 1.0 / accelNoiseScale;

        size_t numSkipped = 0;

        boost::shared_ptr<aslam::backend::MEstimator> mest;
        if (mSigma > 0.0) {
            mest = std::make_unique<aslam::backend::HuberMEstimator>(mSigma);
        } else {
            mest = std::make_unique<aslam::backend::NoMEstimator>();
        }

        for (const auto& im : *imuData) {
            const auto tk = im.stamp + timeOffset;
            if (tk > poseSplineDv->spline().t_min() && tk < poseSplineDv->spline().t_max()) {
                const auto C_b_w = poseSplineDv->orientation(tk).inverse();
                const auto a_w = poseSplineDv->linearAcceleration(tk);
                const auto b_i = accelBiasDv->toEuclideanExpression(tk, 0);
                const auto w_b = poseSplineDv->angularVelocityBodyFrame(tk);
                const auto w_dot_b = poseSplineDv->angularAccelerationBodyFrame(tk);
                const auto C_i_b = q_i_b_Dv->toExpression();
                const auto r_b = r_b_Dv->toExpression();
                const auto a = C_i_b * (C_b_w * (a_w - g_w) + w_dot_b.cross(r_b) + w_b.cross(w_b.cross(r_b)));
                auto aerr
                    = boost::make_shared<kalibr_errorterms::EuclideanError>(im.alpha, im.alphaInvR * weight, a + b_i);
                aerr->setMEstimatorPolicy(mest);
                accelErrors.push_back(aerr);
                problem->addErrorTerm(aerr);
            } else {
                ++numSkipped;
            }
        }

        std::cout << "  Added " << imuData->size() - numSkipped << " of " << imuData->size()
                  << " accelerometer error terms "
                  << "(skipped " << numSkipped << " out-of-bounds measurements)" << std::endl;
    }

    void addGyroscopeErrorTerms(
        boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
        boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseSplineDv,
        const Eigen::Vector3d& g_w,
        double mSigma = 0.0,
        double gyroNoiseScale = 1.0) {
        std::cout << std::endl << "Adding gyroscope error terms (" << imuData->size() << ") ..." << std::endl;

        const double weight = 1.0 / gyroNoiseScale;

        size_t numSkipped = 0;

        boost::shared_ptr<aslam::backend::MEstimator> mest;
        if (mSigma > 0.0) {
            mest = std::make_unique<aslam::backend::HuberMEstimator>(mSigma);
        } else {
            mest = std::make_unique<aslam::backend::NoMEstimator>();
        }

        for (const auto& im : *imuData) {
            const auto tk = im.stamp + timeOffset;
            if (tk > poseSplineDv->spline().t_min() && tk < poseSplineDv->spline().t_max()) {
                const auto w_b = poseSplineDv->angularVelocityBodyFrame(tk);
                const auto b_i = gyroBiasDv->toEuclideanExpression(tk, 0);
                const auto C_i_b = q_i_b_Dv->toExpression();
                const auto w = C_i_b * w_b;
                auto gerr
                    = boost::make_shared<kalibr_errorterms::EuclideanError>(im.omega, im.omegaInvR * weight, w + b_i);
                gerr->setMEstimatorPolicy(mest);
                gyroErrors.push_back(gerr);
                problem->addErrorTerm(gerr);
            } else {
                ++numSkipped;
            }
        }

        std::cout << "  Added " << imuData->size() - numSkipped << " of " << imuData->size()
                  << " gyroscope error terms "
                  << "(skipped " << numSkipped << " out-of-bounds measurements)" << std::endl;
    }

    void initBiasSplines(
        boost::shared_ptr<bsplines::BSplinePose> poseSpline,
        size_t splineOrder,
        size_t biasKnotsPerSecond) {
        const auto start = poseSpline->t_min();
        const auto end = poseSpline->t_max();
        const auto seconds = end - start;
        const auto knots = static_cast<size_t>(round(seconds * static_cast<double>(biasKnotsPerSecond)));

        std::cout << std::endl << "Initializing the bias splines with " << knots << " knots" << std::endl;

        // initialize the bias splines
        gyroBias = boost::make_shared<bsplines::BSpline>(splineOrder);
        gyroBias->initConstantSpline(start, end, knots, gyroBiasPrior);

        accelBias = boost::make_shared<bsplines::BSpline>(splineOrder);
        accelBias->initConstantSpline(start, end, knots, Eigen::Vector3d(0., 0., 0.));
    }

    void addBiasMotionTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem) {
        const auto Wgyro = Eigen::Matrix3d::Identity() / (gyroRandomWalk * gyroRandomWalk);
        const auto Waccel = Eigen::Matrix3d::Identity() / (accelRandomWalk * accelRandomWalk);
        const auto gyroBiasMotionErr
            = boost::make_shared<aslam::backend::BSplineMotionError<aslam::splines::EuclideanBSplineDesignVariable>>(
                gyroBiasDv.get(),
                Wgyro,
                1);
        problem->addErrorTerm(gyroBiasMotionErr);
        const auto accelBiasMotionErr
            = boost::make_shared<aslam::backend::BSplineMotionError<aslam::splines::EuclideanBSplineDesignVariable>>(
                accelBiasDv.get(),
                Waccel,
                1);
        problem->addErrorTerm(accelBiasMotionErr);
    }

    double getMeanGyroscopeError() {
        std::vector<double> gyroVals;
        gyroVals.reserve(gyroErrors.size());

        for (const auto& err : gyroErrors) {
            gyroVals.push_back(err->error().norm());
        }
        const auto [mean, median, std] = errorStatistics(gyroVals);
        return mean;
    }

    double getMeanAccelerometerError() {
        std::vector<double> accelVals;
        accelVals.reserve(accelErrors.size());

        for (const auto& err : accelErrors) {
            accelVals.push_back(err->error().norm());
        }

        const auto [mean, median, std] = errorStatistics(accelVals);

        return mean;
    }

    void printNormalizedResiduals(std::ostream& ss) {
        std::vector<double> gyroVals, accelVals;
        gyroVals.reserve(gyroErrors.size());
        accelVals.reserve(accelErrors.size());

        for (const auto& err : gyroErrors) {
            gyroVals.push_back(err->evaluateError());
        }
        {
            const auto [mean, median, std] = errorStatistics(gyroVals);
            ss << "Gyroscope error:       mean: " << mean << " median: " << median << " std: " << std << std::endl;
        }

        for (const auto& err : accelErrors) {
            accelVals.push_back(err->evaluateError());
        }
        {
            const auto [mean, median, std] = errorStatistics(accelVals);
            ss << "Accelerometer error:   mean: " << mean << " median: " << median << " std: " << std << std::endl;
        }
    }

    void printResiduals(std::ostream& ss) {
        std::vector<double> gyroVals, accelVals;
        gyroVals.reserve(gyroErrors.size());
        accelVals.reserve(accelErrors.size());

        for (const auto& err : gyroErrors) {
            gyroVals.push_back(err->error().norm());
        }
        {
            const auto [mean, median, std] = errorStatistics(gyroVals);
            ss << "Gyroscope error [rad/s]:      mean: " << mean << " median: " << median << " std: " << std
               << std::endl;
        }

        for (const auto& err : accelErrors) {
            accelVals.push_back(err->error().norm());
        }
        {
            const auto [mean, median, std] = errorStatistics(accelVals);
            ss << "Accelerometer error [m/s^2]:  mean: " << mean << " median: " << median << " std: " << std
               << std::endl;
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const IccImu& imu) {
        os << "IMU settings:" << std::endl;
        os << "  Update rate: " << imu.imuParameters.updateRate << std::endl;
        os << "  Accelerometer: " << std::endl;
        os << "    Noise density: " << imu.accelUncertainty << std::endl;
        os << "    Noise density (discrete): " << imu.accelUncertaintyDiscrete << std::endl;
        os << "    Random walk: " << imu.accelRandomWalk << std::endl;
        os << "  Gyroscope: " << std::endl;
        os << "    Noise density: " << imu.gyroUncertainty << std::endl;
        os << "    Noise density (discrete): " << imu.gyroUncertaintyDiscrete << std::endl;
        os << "    Random walk: " << imu.gyroRandomWalk << std::endl;
        return os;
    }
};
