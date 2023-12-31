#pragma once

#include "iccImu.hpp"

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

namespace IccCameraUtils {
using ObservationsPtr = boost::shared_ptr<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>>;
}

template<typename CameraGeometryType, typename DistortionType>
class IccCamera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    const double cornerUncertainty;
    sm::kinematics::Transformation T_extrinsic;
    double timeshiftCamToImuPrior = 0.0;
    CameraModel<CameraGeometryType, DistortionType> camera;
    const cv::Size imageSize;

    Eigen::Vector3d gravity_w;
    double timeOffset = 0.0;

    // Design variables
    boost::shared_ptr<aslam::backend::RotationQuaternion> T_c_b_Dv_q = nullptr;
    boost::shared_ptr<aslam::backend::EuclideanPoint> T_c_b_Dv_t = nullptr;
    boost::shared_ptr<aslam::backend::TransformationBasic> T_c_b_Dv = nullptr;
    boost::shared_ptr<aslam::backend::Scalar> cameraTimeToImuTimeDv = nullptr;

    // Observations of the calibration target
    boost::shared_ptr<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>> targetObservations = nullptr;

    // Error statistics
    typedef aslam::backend::SimpleReprojectionError<aslam::Frame<CameraGeometryType>> ReprojectionError;
    typedef boost::shared_ptr<ReprojectionError> ReprojectionErrorSP;
    std::vector<std::vector<ReprojectionErrorSP>> allReprojectionErrors;

public:
    IccCamera(
        const std::vector<double>& intrinsics,
        const std::vector<double>& distCoeffs,
        const cv::Size& _imageSize,
        boost::shared_ptr<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>> observations,
        const double reprojectionSigma = 1.0,
        const bool showCorners = true,
        const bool showReproj = true,
        const bool showOneStep = false) :
        camera(intrinsics, distCoeffs, _imageSize),
        imageSize(imageSize), cornerUncertainty(reprojectionSigma) {
        targetObservations = observations;
        gravity_w = Eigen::Vector3d(9.80655, 0., 0.);
        T_extrinsic = sm::kinematics::Transformation();

        camera.printDetails();
    }

    void updateIntrinsics(const std::vector<double>& _intrinsics, const std::vector<double>& _distCoeffs) {
        camera = CameraModel<CameraGeometryType, DistortionType>(_intrinsics, _distCoeffs, imageSize);
    }

    sm::kinematics::Transformation getTransformation() {
        assert(T_c_b_Dv_q != nullptr);
        assert(T_c_b_Dv_t != nullptr);
        assert(T_c_b_Dv != nullptr);

        return sm::kinematics::Transformation(T_c_b_Dv_q->getQuaternion(), T_c_b_Dv_t->toEuclidean());
    }

    double getResultTimeShift() {
        return cameraTimeToImuTimeDv->toScalar();
    }

    void findOrientationPriorCameraToImu(boost::shared_ptr<IccImu> iccImu) {
        std::cout << std::endl << "Estimating imu-camera rotation prior" << std::endl << std::endl;

        // Build the problem
        auto problem = boost::make_shared<aslam::backend::OptimizationProblem>();

        // Add the rotation as design variable
        auto q_i_c_Dv = boost::make_shared<aslam::backend::RotationQuaternion>(T_extrinsic.q());
        q_i_c_Dv->setActive(true);
        problem->addDesignVariable(q_i_c_Dv);

        // Add the gyro bias as design variable
        auto gyroBiasDv = boost::make_shared<aslam::backend::EuclideanPoint>(Eigen::Vector3d(0.0, 0.0, 0.0));
        gyroBiasDv->setActive(true);
        problem->addDesignVariable(gyroBiasDv);

        // Initialize a pose spline using the camera poses
        auto poseSpline = initPoseSplineFromCamera(6, 100, 0.0);

        for (const auto& im : iccImu->getImuData()) {
            const auto tk = im.stamp;

            if (tk > poseSpline->t_min() && tk < poseSpline->t_max()) {
                // DV expressions
                const auto R_i_c = q_i_c_Dv->toExpression();
                const auto bias = gyroBiasDv->toExpression();

                // get the vision predicted omega and measured omega (IMU)
                const auto omega_predicted
                    = R_i_c
                      * aslam::backend::EuclideanExpression((poseSpline->angularVelocityBodyFrame(tk)).transpose());
                const auto omega_measured = im.omega;

                // error term
                auto gerr = boost::make_shared<kalibr_errorterms::GyroscopeError>(
                    omega_measured,
                    im.omegaInvR,
                    omega_predicted,
                    bias);
                problem->addErrorTerm(gerr);
            }
        }

        if (problem->numErrorTerms() == 0) {
            throw std::runtime_error("Failed to obtain orientation prior."
                                     "Please make sure that your sensors are synchronized correctly.");
        }

        // Define the optimization
        aslam::backend::Optimizer2Options options;
        options.verbose = false;
        options.linearSystemSolver = boost::make_shared<aslam::backend::BlockCholeskyLinearSystemSolver>();
        options.nThreads = 2;
        options.convergenceDeltaX = 1e-4;
        options.convergenceDeltaJ = 1;
        options.maxIterations = 50;

        // run the optimization
        aslam::backend::Optimizer2 optimizer(options);
        optimizer.setProblem(problem);

        try {
            auto retval = optimizer.optimize();
            if (retval.iterations == options.maxIterations) {
                std::cout << "WARNING: Failed to find orientation prior" << std::endl;
            }
        } catch (...) {
            throw std::runtime_error("Failed to obtain orientation prior!");
        }

        // overwrite the external rotation prior (keep the external translation prior)
        const auto R_i_c = q_i_c_Dv->toRotationMatrix().transpose();
        T_extrinsic = sm::kinematics::Transformation(sm::kinematics::rt2Transform(R_i_c, T_extrinsic.t()));

        // estimate gravity in the world coordinate frame as the mean specific force
        std::vector<Eigen::Vector3d> a_w;
        a_w.reserve(iccImu->getImuData().size());
        for (const auto& im : iccImu->getImuData()) {
            const auto tk = im.stamp;
            if (tk > poseSpline->t_min() && tk < poseSpline->t_max()) {
                const auto val = poseSpline->orientation(tk) * R_i_c * (-im.alpha);
                a_w.emplace_back(val);
            }
        }

        assert(!a_w.empty());
        Eigen::Vector3d sum(0., 0., 0.);
        for (const auto& a : a_w) {
            sum += a;
        }
        const auto mean_a_w = sum / static_cast<double>(a_w.size());
        gravity_w = mean_a_w / mean_a_w.norm() * 9.80655;
        std::cout << "Gravity was intialized to " << gravity_w.transpose() << " [m/s^2]" << std::endl;

        // set the gyro bias prior (if we have more than 1 cameras use recursive average)
        const auto b_gyro = gyroBiasDv->toExpression().toEuclidean();
        iccImu->gyroBiasPriorCount++;
        iccImu->gyroBiasPrior = (iccImu->gyroBiasPriorCount - 1.0) / iccImu->gyroBiasPriorCount * iccImu->gyroBiasPrior
                                + 1.0 / iccImu->gyroBiasPriorCount * b_gyro;

        // print result
        std::cout << " Transformation prior camera-imu found as: (T_extrinsic)" << std::endl;
        std::cout << T_extrinsic.T() << std::endl;
        std::cout << "  Gyro bias prior found as: (b_gyro)" << std::endl;
        std::cout << b_gyro.transpose() << std::endl << std::endl;
    }

    boost::shared_ptr<aslam::cameras::CameraGeometryBase> getCameraGeometry() {
        return camera.getGeometry();
    }

    Eigen::Vector3d getEstimatedGravity() {
        return gravity_w;
    }

    void findTimeshiftCameraImuPrior(boost::shared_ptr<IccImu> iccImu, bool verbose) {
        auto poseSpline = initPoseSplineFromCamera(6, 100, 0.0);

        std::vector<double> times, omega_measured_norm, omega_predicted_norm;
        times.reserve(iccImu->getImuData().size());
        omega_measured_norm.reserve(iccImu->getImuData().size());
        omega_predicted_norm.reserve(iccImu->getImuData().size());
        for (const auto& im : iccImu->getImuData()) {
            const auto tk = im.stamp;
            if (tk > poseSpline->t_min() && tk < poseSpline->t_max()) {
                const auto omega_measured = im.omega;
                const auto omega_predicted
                    = aslam::backend::EuclideanExpression(poseSpline->angularVelocityBodyFrame(tk).transpose());

                times.push_back(tk);
                omega_measured_norm.push_back(omega_measured.norm());
                omega_predicted_norm.push_back(omega_predicted.toEuclidean().norm());
            }
        }

        if (omega_measured_norm.empty() || omega_predicted_norm.empty()) {
            throw std::runtime_error("The time ranges of the camera and IMU do not overlap. "
                                     "Please make sure that your sensors are synchronized correctly.");
        }

        // Get the time shift
        Eigen::VectorXd measured(omega_measured_norm.size());
        Eigen::VectorXd predicted(omega_predicted_norm.size());
        for (size_t i = 0; i < omega_measured_norm.size(); ++i) {
            measured[i] = omega_measured_norm[i];
        }
        for (size_t i = 0; i < omega_predicted_norm.size(); ++i) {
            predicted[i] = omega_predicted_norm[i];
        }

        const auto paddedSize = measured.size() + 2 * predicted.size() - 2;
        Eigen::VectorXd zeroPaddedMeasurement(paddedSize);
        zeroPaddedMeasurement.setZero();
        zeroPaddedMeasurement.segment(predicted.size() / 2, measured.size()) = measured;

        double maxVal = -1;
        size_t maxIdx = 0;
        for (size_t i = 0; i < paddedSize - predicted.size(); ++i) {
            Eigen::VectorXd measuredSlice = zeroPaddedMeasurement.segment(i, predicted.size());
            double val = measuredSlice.transpose() * predicted;
            if (val > maxVal) {
                maxVal = val;
                maxIdx = i;
            }
        }

        const auto discrete_shift = static_cast<int>(maxIdx) - predicted.size() / 2;

        double dt = 0;
        for (size_t i = 0; i < times.size() - 1; ++i) {
            dt += times[i + 1] - times[i];
        }
        dt /= static_cast<double>(times.size() - 1);
        double shift = -static_cast<double>(discrete_shift) * dt;

        timeshiftCamToImuPrior = shift;

        std::cout << "  Time shift camera to imu (t_imu = t_cam + shift): " << std::endl;
        std::cout << "  " << timeshiftCamToImuPrior << std::endl;
    }

    boost::shared_ptr<bsplines::BSplinePose> initPoseSplineFromCamera(
        const size_t splineOrder = 6,
        const size_t poseKnotsPerSecond = 100,
        const double timeOffsetPadding = 0.02) {
        assert(!targetObservations->empty());

        const auto T_c_b = T_extrinsic.T();
        auto rotationVector = boost::make_shared<sm::kinematics::RotationVector>();
        auto pose = boost::make_shared<bsplines::BSplinePose>(splineOrder, rotationVector);

        // Get the checkerboard times.
        std::vector<double> timesVec;
        timesVec.reserve(targetObservations->size());
        std::vector<Eigen::VectorXd> curveVec;
        curveVec.reserve(targetObservations->size());
        for (const auto& [ts, targetObs] : *targetObservations) {
            timesVec.push_back(targetObs.time().toSec() + timeshiftCamToImuPrior);
            const auto trans = targetObs.T_t_c().T() * T_c_b;
            const auto column = pose->transformationToCurveValue(trans);
            curveVec.push_back(column);
        }

        Eigen::VectorXd times(targetObservations->size() + 2);
        times.setZero();
        Eigen::Matrix<double, 6, Eigen::Dynamic> curve(6, targetObservations->size() + 2);
        curve.setZero();

        auto isNan = [](const Eigen::MatrixXd& mat) {
            return !(mat.array() == mat.array()).all();
        };

        // Add 2 seconds on either end to allow the spline to slide during optimization
        times[0] = timesVec.front() - (timeOffsetPadding * 2.0);
        curve.block(0, 0, 6, 1) = curveVec.front();
        times[static_cast<int>(targetObservations->size() + 1)] = timesVec.back() + (timeOffsetPadding * 2.0);
        curve.block(0, static_cast<int>(targetObservations->size() + 1), 6, 1) = curveVec.back();
        for (int idx = 0; idx < targetObservations->size(); ++idx) {
            times[idx + 1] = timesVec[static_cast<size_t>(idx)];
            curve.block(0, idx + 1, 6, 1) = curveVec[static_cast<size_t>(idx)];
        }

        if (isNan(curve)) {
            throw std::runtime_error("NaNs in curve values");
        }

        // Make sure the rotation vector doesn't flip
        for (int i = 1; i < curve.cols(); ++i) {
            const Eigen::MatrixXd previousRotationVector = curve.block(3, i - 1, 3, 1);
            const Eigen::MatrixXd r = curve.block(3, i, 3, 1);
            const auto angle = r.norm();
            const auto axis = r / angle;

            auto best_r = r;
            auto best_dist = (best_r - previousRotationVector).norm();
            for (int s = -3; s < 4; ++s) {
                const auto aa = axis * (angle + M_PI * 2.0 * s);
                const auto dist = (aa - previousRotationVector).norm();
                if (dist < best_dist) {
                    best_r = aa;
                    best_dist = dist;
                }
            }
            curve.block(3, i, 3, 1) = best_r;
        }

        const auto seconds = static_cast<double>(times[times.size() - 1] - times[0]);
        const auto knots = static_cast<int>(std::round(seconds * poseKnotsPerSecond));

        std::cout << "Initializing a pose spline with " << knots << " knots (" << poseKnotsPerSecond
                  << " knots per second over " << seconds << " seconds)" << std::endl;
        pose->initPoseSplineSparse(times, curve, knots, 1e-4);
        return pose;
    }

    void addDesignVariables(
        boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
        bool noExtrinsics = true,
        bool noTimeCalibration = true,
        size_t baselinedv_group_id = HELPER_GROUP_ID) {
        T_c_b_Dv_q = boost::make_shared<aslam::backend::RotationQuaternion>(T_extrinsic.q());
        T_c_b_Dv_q->setActive(true);
        problem->addDesignVariable(T_c_b_Dv_q, baselinedv_group_id);

        T_c_b_Dv_t = boost::make_shared<aslam::backend::EuclideanPoint>(T_extrinsic.t());
        T_c_b_Dv_t->setActive(true);
        problem->addDesignVariable(T_c_b_Dv_t, baselinedv_group_id);

        T_c_b_Dv = boost::make_shared<aslam::backend::TransformationBasic>(
            T_c_b_Dv_q->toExpression(),
            T_c_b_Dv_t->toExpression());

        cameraTimeToImuTimeDv = boost::make_shared<aslam::backend::Scalar>(0.0);
        cameraTimeToImuTimeDv->setActive(!noTimeCalibration);
        problem->addDesignVariable(cameraTimeToImuTimeDv, CALIBRATION_GROUP_ID);
    }

    void addCameraErrorTerms(
        boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
        boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseSplineDv,
        double blakeZissermanDf = 0.0,
        double timeOffsetPadding = 0.0) {
        std::cout << std::endl << "Adding camera error terms (" << targetObservations->size() << ") ..." << std::endl;

        const auto T_cN_b = T_c_b_Dv->toExpression();

        for (const auto& [ts, obs] : *targetObservations) {
            const auto frameTime = cameraTimeToImuTimeDv->toExpression() + obs.time().toSec() + timeshiftCamToImuPrior;
            const auto frameTimeScalar = frameTime.toScalar();

            // #as we are applying an initial time shift outside the optimization so
            // #we need to make sure that we dont add data outside the spline definition
            if (frameTimeScalar <= poseSplineDv->spline().t_min()
                || frameTimeScalar >= poseSplineDv->spline().t_max()) {
                continue;
            }

            const auto T_w_b = poseSplineDv->transformationAtTime(frameTime, timeOffsetPadding, timeOffsetPadding);
            const auto T_b_w = T_w_b.inverse();

            // #calibration target coords to camera N coords
            // #T_b_w: from world to imu coords
            // #T_cN_b: from imu to camera N coords
            const auto T_c_w = T_cN_b * T_b_w;

            // #get the image and target points corresponding to the frame
            std::vector<cv::Point2f> imageCornerPoints;
            size_t nImageCorners = obs.getCornersImageFrame(imageCornerPoints);
            std::vector<cv::Point3f> targetCornerPoints;
            size_t nTargetCorners = obs.getCornersTargetFrame(targetCornerPoints);
            assert(nImageCorners == nTargetCorners);

            // #setup an aslam frame (handles the distortion)
            auto frame = camera.frame();
            frame->setGeometry(camera.getGeometry());

            // #corner uncertainty
            const auto cornerUncVal = cornerUncertainty * cornerUncertainty;
            Eigen::Matrix2d R;
            R.setZero();
            R(0, 0) = cornerUncVal;
            R(1, 1) = cornerUncVal;
            const auto invR = R.inverse();

            for (size_t idx = 0; idx < imageCornerPoints.size(); ++idx) {
                const auto& imageCornerPoint = imageCornerPoints[idx];

                // Image points
                aslam::Keypoint<2> k;
                Eigen::Matrix<double, 2, 1> mat;
                mat.setZero();
                mat(0, 0) = imageCornerPoint.x;
                mat(1, 0) = imageCornerPoint.y;
                k.setMeasurement(mat);
                k.setInverseMeasurementCovariance(invR);
                frame->addKeypoint(k);
            }

            std::vector<ReprojectionErrorSP> reprojectionErrors;
            reprojectionErrors.reserve(imageCornerPoints.size());
            for (size_t idx = 0; idx < imageCornerPoints.size(); ++idx) {
                const auto& imageCornerPoint = imageCornerPoints[idx];
                const auto& targetPoint = targetCornerPoints[idx];

                // Target points
                const Eigen::Vector3d eigenPoint(
                    static_cast<double>(targetPoint.x),
                    static_cast<double>(targetPoint.y),
                    static_cast<double>(targetPoint.z));
                const auto p = T_c_w * aslam::backend::HomogeneousExpression(eigenPoint);

                // #build and append the error term
                auto rerr = boost::make_shared<ReprojectionError>(frame.get(), idx, p);

                // #add blake-zisserman m-estimator
                if (blakeZissermanDf > 0.0) {
                    throw std::runtime_error("Not implemented blake-zisserman");
                    // mest = aopt.BlakeZissermanMEstimator( blakeZissermanDf )
                    // rerr.setMEstimatorPolicy(mest)
                }

                problem->addErrorTerm(rerr);
                reprojectionErrors.push_back(rerr);
            }
            allReprojectionErrors.push_back(reprojectionErrors);
        }

        std::cout << "  Added " << targetObservations->size() << " camera error tems" << std::endl;
    }

    double getMeanReprojectionError() {
        std::vector<double> errVals;
        for (const auto& vec : allReprojectionErrors) {
            for (const auto& val : vec) {
                errVals.push_back(val->error().norm());
            }
        }

        const auto [mean, median, std] = errorStatistics(errVals);

        return mean;
    }

    void printNormalizedResiduals(std::ostream& ss) {
        if (allReprojectionErrors.empty()) {
            ss << "Reprojection error:    no corners" << std::endl;
        }

        std::vector<double> errVals;
        for (const auto& vec : allReprojectionErrors) {
            for (const auto& val : vec) {
                errVals.push_back(val->evaluateError());
            }
        }

        const auto [mean, median, std] = errorStatistics(errVals);
        ss << "Reprojection error:    mean: " << mean << " median: " << median << " std: " << std << std::endl;
    }

    void printResiduals(std::ostream& ss) {
        if (allReprojectionErrors.empty()) {
            ss << "Reprojection error [px]:    no corners" << std::endl;
        }

        std::vector<double> errVals;
        for (const auto& vec : allReprojectionErrors) {
            for (const auto& val : vec) {
                errVals.push_back(val->error().norm());
            }
        }

        const auto [mean, median, std] = errorStatistics(errVals);
        ss << "Reprojection error [px]:      mean: " << mean << " median: " << median << " std: " << std << std::endl;
    }

    std::ostream& print(std::ostream& os) {
        return camera.print(os);
    }
};
