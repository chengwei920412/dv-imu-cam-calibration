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
    const sm::kinematics::Transformation& T0) {
    auto q_Dv = boost::make_shared<aslam::backend::RotationQuaternion>(T0.q());
    q_Dv->setActive(true);
    problem->addDesignVariable(q_Dv);

    auto t_Dv = boost::make_shared<aslam::backend::EuclideanPoint>(T0.t());
    t_Dv->setActive(true);
    problem->addDesignVariable(t_Dv);

    return boost::make_shared<aslam::backend::TransformationBasic>(q_Dv->toExpression(), t_Dv->toExpression());
}

std::tuple<double, double> meanStd(std::vector<double> vals) {
    double sum = 0.0;
    for (const auto& val : vals) {
        sum += val;
    }
    double mean = sum / static_cast<double>(vals.size());

    double stdSum = 0.0;
    for (const auto& val : vals) {
        double diff = val - mean;
        stdSum += diff * diff;
    }
    double std = sqrt(stdSum / (static_cast<double>(vals.size() - 1)));

    return std::make_tuple(mean, std);
}

std::tuple<Eigen::Vector2d, Eigen::Vector2d>
    getReprojectionErrorStatistics(const std::vector<std::vector<Eigen::MatrixXd>>& all_rerrs) {
    std::vector<double> xVals, yVals;
    for (const auto& view_rerrs : all_rerrs) {
        if (view_rerrs.empty()) {
            continue;
        }

        for (const auto& rerr : view_rerrs) {
            if (rerr.size() == 0) {
                continue;
            }

            xVals.push_back(rerr(0, 0));
            yVals.push_back(rerr(1, 0));
        }
    }

    const auto [xMean, xStd] = meanStd(xVals);
    const auto [yMean, yStd] = meanStd(yVals);

    return std::make_tuple(Eigen::Vector2d(xMean, yMean), Eigen::Vector2d(xStd, yStd));
}

template<typename CameraGeometryType, typename DistortionType>
class CameraGeometry {
private:
    boost::shared_ptr<IccCamera<CameraGeometryType, DistortionType>> iccCamera = nullptr;
    boost::shared_ptr<aslam::backend::CameraDesignVariable<CameraGeometryType>> dv = nullptr;
    bool isGeometryInitialized;

public:
    CameraGeometry(boost::shared_ptr<IccCamera<CameraGeometryType, DistortionType>> camera) {
        iccCamera = camera;

        auto geometry = camera->getCameraGeometry();

        auto casted = boost::dynamic_pointer_cast<CameraGeometryType>(geometry);
        if (casted == nullptr) {
            throw std::runtime_error("Something went wrong with casting of the pointer of camera geometry");
        }

        // create the design variables
        dv = boost::make_shared<aslam::backend::CameraDesignVariable<CameraGeometryType>>(casted);
        setDvActiveStatus(true, true, false);
        isGeometryInitialized = false;
    }

    bool initGeometryFromObservations(
        boost::shared_ptr<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>> observationsMap,
        boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target) {
        // Obtain focal length guess
        auto geometry = iccCamera->getCameraGeometry();
        std::vector<aslam::cameras::GridCalibrationTargetObservation> observationsVector;
        observationsVector.reserve(observationsMap->size());
        for (const auto& [_, obs] : *observationsMap) {
            observationsVector.emplace_back(obs);
        }
        auto success = geometry->initializeIntrinsics(observationsVector);
        if (!success) {
            std::cout << "Initialization of focal length failed" << std::endl;
        }

        // Optimize for intrinsics and distortion
        success = calibrateIntrinsics(observationsVector, target);
        if (!success) {
            // nothing to print
        }

        isGeometryInitialized = success;
        return success;
    }

    bool getIsGeometryInitialized() {
        return isGeometryInitialized;
    }

    boost::shared_ptr<aslam::backend::CameraDesignVariable<CameraGeometryType>> getDv() {
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
        bool intrinsicsActive = true) {
        // verbose output
        Eigen::MatrixXd params;
        iccCamera->getCameraGeometry()->getParameters(params, true, true, true);
        std::cout << "calibrateIntrinsics: intrinsics guess: " << params.block<4, 1>(0, 0).transpose() << std::endl;
        std::cout << "calibrateIntrinsics: distortion guess: " << params.block(4, 0, params.rows() - 4, 1).transpose()
                  << std::endl;

        // ############################################
        // ## solve the bundle adjustment
        // ############################################
        auto problem = boost::make_shared<aslam::backend::OptimizationProblem>();

        // add camera dvs
        setDvActiveStatus(intrinsicsActive, distortionActive, false);
        problem->addDesignVariable(dv->distortionDesignVariable());
        problem->addDesignVariable(dv->projectionDesignVariable());
        problem->addDesignVariable(dv->shutterDesignVariable());

        // corner uncertainty
        const double cornerUncertainty = 1.0;
        const auto R = Eigen::Matrix2d::Identity() * cornerUncertainty * cornerUncertainty;
        const auto invR = R.inverse();

        // # target pose dv for all target views (=T_camL_w)
        typedef aslam::backend::ReprojectionError<CameraGeometryType> ReprojectionError;
        std::vector<boost::shared_ptr<ReprojectionError>> reprojectionErrors;
        std::cout << "calibrateIntrinsics: adding camera error terms for " << obslist.size() << " calibration targets"
                  << std::endl;
        std::vector<boost::shared_ptr<aslam::backend::TransformationBasic>> target_pose_dvs;
        target_pose_dvs.reserve(obslist.size());
        for (const auto& obs : obslist) {
            sm::kinematics::Transformation T_t_c;
            iccCamera->getCameraGeometry()->estimateTransformation(obs, T_t_c);
            auto target_pose_dv = addPoseDesignVariable(problem, T_t_c);
            target_pose_dvs.push_back(target_pose_dv);

            const auto T_cam_w = target_pose_dv->toExpression().inverse();

            // add error terms
            for (size_t i = 0; i < target->size(); ++i) {
                auto p_target = aslam::backend::HomogeneousExpression(sm::kinematics::toHomogeneous(target->point(i)));
                Eigen::Vector2d y;
                if (obs.imagePoint(i, y)) {
                    auto rerr = boost::make_shared<ReprojectionError>(y, invR, T_cam_w * p_target, *dv);
                    problem->addErrorTerm(rerr);
                    reprojectionErrors.push_back(rerr);
                }
            }
        }
        std::cout << "calibrateIntrinsics: added " << reprojectionErrors.size() << " camera error terms" << std::endl;

        // ############################################
        // ## solve
        // ############################################
        aslam::backend::Optimizer2Options options;
        options.verbose = false;
        options.nThreads = 4;
        options.convergenceDeltaX = 1e-3;
        options.convergenceDeltaJ = 1;
        options.maxIterations = 200;
        options.trustRegionPolicy = boost::make_shared<aslam::backend::LevenbergMarquardtTrustRegionPolicy>(10);

        aslam::backend::Optimizer2 optimizer(options);
        optimizer.setProblem(problem);

        // verbose output
        auto printReprErrors = [&reprojectionErrors](const std::string& prefix) {
            std::vector<double> vals;
            vals.reserve(reprojectionErrors.size());
            for (const auto& rerr : reprojectionErrors) {
                vals.push_back(rerr->evaluateError());
            }

            const auto [mean, std] = meanStd(vals);
            std::cout << prefix << " mean: " << mean << " std: " << std << std::endl;
        };
        printReprErrors("calibrateIntrinsics: Before Optimization: ");

        // run intrinsic calibration
        bool success = false;
        try {
            auto retval = optimizer.optimize();
            if (retval.linearSolverFailure) {
                std::cout << "calibrateIntrinsics: Optimization failed!" << std::endl;
            }
            success = not retval.linearSolverFailure;
        } catch (...) {
            std::cout << "calibrateIntrinsics: Optimization failed!" << std::endl;
        }

        printReprErrors("calibrateIntrinsics: After Optimization: ");

        iccCamera->getCameraGeometry()->getParameters(params, true, true, true);
        std::cout << "calibrateIntrinsics: optimized intrinsics guess: " << params.block<4, 1>(0, 0).transpose()
                  << std::endl;
        std::cout << "calibrateIntrinsics: optimized distortion guess: "
                  << params.block(4, 0, params.rows() - 4, 1).transpose() << std::endl;

        return success;
    }
};

template<typename CameraGeometryType, typename DistortionType>
class CalibrationTargetOptimizationProblem : public aslam::calibration::OptimizationProblem {
public:
    // arguments
    boost::shared_ptr<CameraGeometry<CameraGeometryType, DistortionType>> camera;
    boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target;
    sm::kinematics::Transformation T_tc_guess;
    aslam::cameras::GridCalibrationTargetObservation rig_observation;

    // others
    boost::shared_ptr<aslam::backend::RotationQuaternion> dv_T_target_camera_q = nullptr;
    boost::shared_ptr<aslam::backend::EuclideanPoint> dv_T_target_camera_t = nullptr;
    boost::shared_ptr<aslam::backend::TransformationBasic> dv_T_target_camera = nullptr;
    std::vector<aslam::backend::HomogeneousExpression> P_t_ex;
    std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>> P_t_dv;
    std::vector<boost::shared_ptr<aslam::backend::ReprojectionError<CameraGeometryType>>> rerrs;

    // constructor is merged with factory method from Python "fromTargetViewObservations"
    CalibrationTargetOptimizationProblem(
        boost::shared_ptr<CameraGeometry<CameraGeometryType, DistortionType>> _camera,
        boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> _target,
        const sm::kinematics::Transformation& _T_tc_guess,
        const aslam::cameras::GridCalibrationTargetObservation& _rig_observation,
        bool estimateLandmarks,
        bool useBlakeZissermanMest) {
        // store the arguments in case we want to rebuild a modified problem
        camera = _camera;
        target = _target;
        T_tc_guess = _T_tc_guess;
        rig_observation = _rig_observation;

        // 1. Create a design variable for this pose
        const auto T_target_camera = T_tc_guess;

        dv_T_target_camera_q = boost::make_shared<aslam::backend::RotationQuaternion>(T_target_camera.q());
        dv_T_target_camera_q->setActive(true);
        this->addDesignVariable(dv_T_target_camera_q, TRANSFORMATION_GROUP_ID);
        dv_T_target_camera_t = boost::make_shared<aslam::backend::EuclideanPoint>(T_target_camera.t());
        dv_T_target_camera_t->setActive(true);
        this->addDesignVariable(dv_T_target_camera_t, TRANSFORMATION_GROUP_ID);

        dv_T_target_camera = boost::make_shared<aslam::backend::TransformationBasic>(
            dv_T_target_camera_q->toExpression(),
            dv_T_target_camera_t->toExpression());

        // 2. Add all baselines DVs
        // No baselines, only monocular calibration supported at the moment

        // 3. Add landmark DVs
        P_t_ex.reserve(target->size());
        P_t_dv.reserve(target->size());
        for (size_t i = 0; i < target->size(); ++i) {
            auto p_t_dv
                = boost::make_shared<aslam::backend::HomogeneousPoint>(sm::kinematics::toHomogeneous(target->point(i)));
            p_t_dv->setActive(estimateLandmarks);
            auto p_t_ex = p_t_dv->toExpression();
            P_t_ex.push_back(p_t_ex);
            P_t_dv.push_back(p_t_dv);

            this->addDesignVariable(p_t_dv, LANDMARK_GROUP_ID);
        }

        // 4. add camera DVs
        if (!camera->getIsGeometryInitialized()) {
            throw std::runtime_error("The camera geometry is not initialized. Please initialize with initGeometry() or "
                                     "initGeometryFromDataset()");
        }
        camera->setDvActiveStatus(true, true, false);
        this->addDesignVariable(camera->getDv()->distortionDesignVariable(), CALIBRATION_GROUP_ID);
        this->addDesignVariable(camera->getDv()->projectionDesignVariable(), CALIBRATION_GROUP_ID);
        this->addDesignVariable(camera->getDv()->shutterDesignVariable(), CALIBRATION_GROUP_ID);

        // 5. add all observations for this view
        size_t rerr_cnt = 0;
        rerrs.reserve(P_t_ex.size());

        double cornerUncertainty = 1.0;
        const auto R = Eigen::Matrix2d::Identity() * cornerUncertainty * cornerUncertainty;
        const auto invR = R.inverse();

        // add reprojection errors
        // build baseline chain (target->cam0->baselines->camN)
        const auto T_cam0_target = dv_T_target_camera->toExpression().inverse();
        auto T_camN_calib = T_cam0_target;
        // no chain to build as we only support mono calibration

        for (size_t i = 0; i < P_t_ex.size(); ++i) {
            const auto p_target = P_t_ex[i];
            Eigen::Vector2d y;
            if (rig_observation.imagePoint(i, y)) {
                ++rerr_cnt;
                // create an error term.
                auto rerr = boost::make_shared<aslam::backend::ReprojectionError<CameraGeometryType>>(
                    y,
                    invR,
                    T_camN_calib * p_target,
                    *camera->getDv());

                if (useBlakeZissermanMest) {
                    throw std::runtime_error("useBlakeZissermanMest not implemented");
                }
                this->addErrorTerm(rerr);
                rerrs.push_back(rerr);
            } else {
                rerrs.emplace_back(nullptr);
            }
        }
    }
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

namespace CameraCalibrationUtils {
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
static void printResult(const CameraCalibrationUtils::CalibrationResult& result, std::stringstream& ss) {
    ss << "Intrinsic calibration results:" << std::endl;
    ss << "  projection: ";
    for (const auto val : result.projection) {
        ss << val << " ";
    }
    ss << std::endl;
    ss << "  distortion: ";
    for (const auto val : result.distortion) {
        ss << val << " ";
    }
    ss << std::endl;

    ss << "  reprojection error: [" << result.err_info.mean.x() << ", " << result.err_info.mean.y() << "] +- ["
       << result.err_info.std.x() << ", " << result.err_info.std.y() << "]" << std::endl;
}
} // namespace CameraCalibrationUtils

template<typename CameraGeometryType, typename DistortionType>
class CameraCalibration {
private:
    boost::shared_ptr<CameraGeometry<CameraGeometryType, DistortionType>> camera = nullptr;
    bool estimateLandmarks;
    bool useBlakeZissermanMest;
    boost::shared_ptr<aslam::calibration::IncrementalEstimator> estimator = nullptr;
    boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target = nullptr;
    std::vector<boost::shared_ptr<CalibrationTargetOptimizationProblem<CameraGeometryType, DistortionType>>> views;

public:
    CameraCalibration(
        boost::shared_ptr<CameraGeometry<CameraGeometryType, DistortionType>> cam,
        boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> _target,
        bool _estimateLandmarks = false,
        bool _useBlakeZissermanMest = true) :
        camera(cam),
        target(_target), estimateLandmarks(_estimateLandmarks), useBlakeZissermanMest(_useBlakeZissermanMest) {
        static constexpr bool verbose = false;

        // create the incremental estimator and set options
        estimator = boost::make_shared<aslam::calibration::IncrementalEstimator>(CALIBRATION_GROUP_ID);
        estimator->getOptions().infoGainDelta = 0.2;
        estimator->getOptions().checkValidity = true;
        estimator->getOptions().verbose = verbose;

        estimator->getLinearSolverOptions().columnScaling = true;
        estimator->getLinearSolverOptions().verbose = verbose;
        estimator->getLinearSolverOptions().epsSVD = 1e-6;

        estimator->getOptimizerOptions().maxIterations = 50;
        estimator->getOptimizerOptions().verbose = verbose;
    }

    bool addTargetView(const aslam::cameras::GridCalibrationTargetObservation& observation, bool force = false) {
        auto T_tc_guess = observation.T_t_c();
        auto batch_problem
            = boost::make_shared<CalibrationTargetOptimizationProblem<CameraGeometryType, DistortionType>>(
                camera,
                target,
                T_tc_guess,
                observation,
                estimateLandmarks,
                useBlakeZissermanMest);
        auto estimator_return_value = estimator->addBatch(batch_problem, force);

        if (estimator_return_value.numIterations >= estimator->getOptimizerOptions().maxIterations) {
            std::cout << "Did not converge in maxIterations..." << std::endl;
            throw OptimizationDiverged("Optimization did not converge in maxIterations");
        }

        bool success = estimator_return_value.batchAccepted;
        if (success) {
            views.push_back(batch_problem);
        }

        return success;
    }

    size_t getNumBatches() {
        return estimator->getNumBatches();
    }

    void replaceBatch(
        const size_t batch_id,
        boost::shared_ptr<CalibrationTargetOptimizationProblem<CameraGeometryType, DistortionType>> new_batch) {
        estimator->removeBatch(views.at(batch_id));
        views.at(batch_id) = new_batch;

        const auto rval = estimator->addBatch(new_batch, false);

        // queue the batch for removal if the corrected batch was rejected
        if (!rval.batchAccepted) {
            views.erase(views.begin() + batch_id);
        }
    }
    std::tuple<
        std::vector<std::vector<Eigen::MatrixXd>>,
        std::vector<std::vector<Eigen::MatrixXd>>,
        std::vector<std::vector<Eigen::MatrixXd>>>
        getReprojectionErrors(const size_t cameraId) {
        assert(cameraId == 0); // only mono supported

        std::vector<std::vector<Eigen::MatrixXd>> all_corners, all_reprojections, all_reprojection_errs;

        for (size_t view_id = 0; view_id < views.size(); ++view_id) {
            const auto view = views.at(view_id);
            // if cam_id in view.rerrs.keys(): // mono, not needed
            std::vector<Eigen::MatrixXd> view_corners, view_reprojections, view_reprojection_errs;
            for (const auto& rerr : view->rerrs) {
                // add if the corners were observed
                Eigen::MatrixXd corner, reprojection, err;
                if (rerr) {
                    corner = rerr->getMeasurement();
                    reprojection = rerr->getPredictedMeasurement();
                    err = corner - reprojection;
                } else {
                    // Nothing, empty matrix
                }

                view_corners.push_back(corner);
                view_reprojections.push_back(reprojection);
                view_reprojection_errs.push_back(err);
            }

            all_corners.push_back(view_corners);
            all_reprojections.push_back(view_reprojections);
            all_reprojection_errs.push_back(view_reprojection_errs);
        }

        return std::make_tuple(all_corners, all_reprojections, all_reprojection_errs);
    }

    CameraCalibrationUtils::CalibrationResult getResult() {
        const auto projectionMat = camera->getDv()->projectionDesignVariable()->getParameters();
        assert(projectionMat.rows() == 4);
        assert(projectionMat.cols() == 1);
        std::vector<double> projection;
        projection.push_back(projectionMat(0, 0));
        projection.push_back(projectionMat(1, 0));
        projection.push_back(projectionMat(2, 0));
        projection.push_back(projectionMat(3, 0));

        const auto distortionMat = camera->getDv()->distortionDesignVariable()->getParameters();
        assert(distortionMat.rows() == 4);
        assert(distortionMat.cols() == 1);
        std::vector<double> distortion;
        distortion.push_back(distortionMat(0, 0));
        distortion.push_back(distortionMat(1, 0));
        distortion.push_back(distortionMat(2, 0));
        distortion.push_back(distortionMat(3, 0));

        // reproj error statistics
        CameraCalibrationUtils::ErrorInfo err_info(Eigen::Vector2d(-1., -1.), Eigen::Vector2d(-1., -1.));
        const auto [corners, reprojs, rerrs] = getReprojectionErrors(0);
        if (!rerrs.empty()) {
            const auto [me, std] = getReprojectionErrorStatistics(rerrs);
            err_info = CameraCalibrationUtils::ErrorInfo(me, std);
        }

        return {projection, distortion, err_info};
    }

    boost::shared_ptr<CalibrationTargetOptimizationProblem<CameraGeometryType, DistortionType>> removeCornersFromBatch(
        const size_t batch_id,
        const std::vector<size_t>& cornerIdList,
        const bool useBlakeZissermanMest = true) {
        auto& batch = views.at(batch_id);

        // disable the corners
        bool hasCornerRemoved = false;
        for (const auto& cornerId : cornerIdList) {
            batch->rig_observation.removeImagePoint(cornerId);
            hasCornerRemoved = true;
        }
        assert(hasCornerRemoved);

        // rebuild problem
        auto new_problem = boost::make_shared<CalibrationTargetOptimizationProblem<CameraGeometryType, DistortionType>>(
            batch->camera,
            batch->target,
            batch->T_tc_guess,
            batch->rig_observation,
            estimateLandmarks,
            useBlakeZissermanMest);
        return new_problem;
    }
};
