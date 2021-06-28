//
// Created by radam on 2021-06-18.
//

#pragma once

#include <kalibr_imu_camera_calibration/iccSensors.hpp>

#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/backend/HomogeneousPoint.hpp>
#include <aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/ReprojectionError.hpp>
#include <aslam/calibration/core/IncrementalEstimator.h>

boost::shared_ptr<aslam::backend::TransformationBasic> addPoseDesignVariable(
    boost::shared_ptr<aslam::backend::OptimizationProblem>& problem,
    const sm::kinematics::Transformation& T0);

std::tuple<double, double> meanStd(std::vector<double> vals);

std::tuple<Eigen::Vector2d, Eigen::Vector2d>
    getReprojectionErrorStatistics(const std::vector<std::vector<Eigen::MatrixXd>>& all_rerrs);

class CameraGeometry {
private:
    boost::shared_ptr<IccCamera> iccCamera = nullptr;
    boost::shared_ptr<aslam::backend::CameraDesignVariable<aslam::cameras::DistortedPinholeCameraGeometry>> dv
        = nullptr;
    bool isGeometryInitialized;

public:
    CameraGeometry(boost::shared_ptr<IccCamera> camera);

    bool initGeometryFromObservations(
        boost::shared_ptr<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>> observationsMap,
        boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target);

    bool getIsGeometryInitialized() {
        return isGeometryInitialized;
    }

    boost::shared_ptr<aslam::backend::CameraDesignVariable<aslam::cameras::DistortedPinholeCameraGeometry>> getDv() {
        return dv;
    }

    void setDvActiveStatus(bool projectionActive, bool distortionActive, bool shutterActive) {
        dv->projectionDesignVariable()->setActive(projectionActive);
        dv->distortionDesignVariable()->setActive(distortionActive);
        dv->shutterDesignVariable()->setActive(shutterActive);
    }

protected:
    // This function Python equivalent can be found in:
    // thirdparty/kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraIntializers.py
    // It was moved to avoid circular dependencies
    bool calibrateIntrinsics(
        const std::vector<aslam::cameras::GridCalibrationTargetObservation>& obslist,
        boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target,
        bool distortionActive = true,
        bool intrinsicsActive = true);
};

class CalibrationTargetOptimizationProblem : public aslam::calibration::OptimizationProblem {
public:
    // arguments
    boost::shared_ptr<CameraGeometry> camera;
    boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target;
    sm::kinematics::Transformation T_tc_guess;
    aslam::cameras::GridCalibrationTargetObservation rig_observation;

    // others
    boost::shared_ptr<aslam::backend::RotationQuaternion> dv_T_target_camera_q = nullptr;
    boost::shared_ptr<aslam::backend::EuclideanPoint> dv_T_target_camera_t = nullptr;
    boost::shared_ptr<aslam::backend::TransformationBasic> dv_T_target_camera = nullptr;
    std::vector<aslam::backend::HomogeneousExpression> P_t_ex;
    std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>> P_t_dv;
    std::vector<boost::shared_ptr<aslam::backend::ReprojectionError<aslam::cameras::DistortedPinholeCameraGeometry>>>
        rerrs;

    // constructor is merged with factory method from Python "fromTargetViewObservations"
    CalibrationTargetOptimizationProblem(
        boost::shared_ptr<CameraGeometry> _camera,
        boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> _target,
        const sm::kinematics::Transformation& _T_tc_guess,
        const aslam::cameras::GridCalibrationTargetObservation& _rig_observation,
        bool estimateLandmarks,
        bool useBlakeZissermanMest);
};

class OptimizationDiverged : public std::exception {
private:
    std::string message_;

public:
    OptimizationDiverged(const std::string& message) : message_(message) {
    }

    const char* what() const noexcept override {
        return message_.c_str();
    }
};

class CameraCalibration {
public:
    struct ErrorInfo {
        Eigen::Vector2d mean;
        Eigen::Vector2d std;

        ErrorInfo(const Eigen::Vector2d& _mean, const Eigen::Vector2d& _std) : mean(_mean), std(_std) {
        }
    };

    struct CalibrationResult {
        const std::vector<double> projection;
        const std::vector<double> distortion;
        const ErrorInfo err_info;

        CalibrationResult(
            const std::vector<double>& _projection,
            const std::vector<double>& _distortion,
            const ErrorInfo& _err_info) :
            projection(_projection),
            distortion(_distortion), err_info(_err_info) {
        }
    };

private:
    boost::shared_ptr<CameraGeometry> camera = nullptr;
    bool estimateLandmarks;
    bool useBlakeZissermanMest;
    boost::shared_ptr<aslam::calibration::IncrementalEstimator> estimator = nullptr;
    boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target = nullptr;
    std::vector<boost::shared_ptr<CalibrationTargetOptimizationProblem>> views;

public:
    CameraCalibration(
        boost::shared_ptr<CameraGeometry> cam,
        boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> _target,
        bool _estimateLandmarks = false,
        bool _useBlakeZissermanMest = true);

    bool addTargetView(const aslam::cameras::GridCalibrationTargetObservation& observation, bool force = false);

    size_t getNumBatches();

    void replaceBatch(const size_t batch_id, boost::shared_ptr<CalibrationTargetOptimizationProblem> new_batch);

    std::tuple<
        std::vector<std::vector<Eigen::MatrixXd>>,
        std::vector<std::vector<Eigen::MatrixXd>>,
        std::vector<std::vector<Eigen::MatrixXd>>>
        getReprojectionErrors(const size_t cameraId);

    CalibrationResult getResult();

    static void printResult(const CalibrationResult& result, std::stringstream& ss);

    boost::shared_ptr<CalibrationTargetOptimizationProblem> removeCornersFromBatch(
        const size_t batch_id,
        const std::vector<size_t>& cornerIdList,
        const bool useBlakeZissermanMest = true);
};
