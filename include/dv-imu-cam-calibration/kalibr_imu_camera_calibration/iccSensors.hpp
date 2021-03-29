//
// Created by radam on 2021-03-26.
//

#pragma once


#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <kalibr_common/ConfigReader.hpp>
#include <kalibr_errorterms/EuclideanError.hpp>

#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/backend/BSplineMotionError.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/OptimizationProblem.hpp>

#include <Eigen/Eigen>

double toSec(const int64_t time);

struct TargetObservation {
  int64_t time;
  sm::kinematics::Transformation T_t_c;
};

class IccImu;

class IccCamera {

protected:
  // TODO(radam): dataset
  // TODO(radam): cam config
  // TODO(radam): target config

  const double cornerUncertainty;
  sm::kinematics::Transformation T_extrinsic;
  double timeshiftCamToImuPrior = 0.0;
  PinholeEquidistantCamera camera{std::vector<double>(),
      std::vector<double>(),
      cv::Size(640, 480)}; // TODO(radam): pass these in constructor
  std::vector<TargetObservation> targetObservations;
  Eigen::Vector3d gravity_w;
  double timeOffset = 0.0;



public:
  IccCamera(const double reprojectionSigma=1.0,
			const bool showCorners=true,
			const bool showReproj = true,
			const bool showOneStep=false); // TODO(radam): params

  // void setupCalibrationTarget // TODO(radam): make sure it is not needed

  void findOrientationPriorCameraToImu(IccImu* iccImu) {
    std::cout << std::endl << "Estimating imu-camera rotation prior" << std::endl;

    // Build the problem
    auto problem = aslam::backend::OptimizationProblem();

    // Add the rotation as design variable
	auto q_i_c_Dv = boost::make_shared<aslam::backend::RotationQuaternion>(T_extrinsic.q());
	q_i_c_Dv->setActive(true);
	problem.addDesignVariable(q_i_c_Dv);

	// Add the gyro bias as design variable
	auto gyroBiasDv = boost::make_shared<aslam::backend::EuclideanPoint>(Eigen::Vector3d(0.0, 0.0, 0.0));
	gyroBiasDv->setActive(true);
	problem.addDesignVariable(gyroBiasDv);

	// Initialize a pose spline using the camera poses
	auto poseSpline = initPoseSplineFromCamera(6, 100, 0.0);



  }

  // TODO(radam): missing methods here

  boost::shared_ptr<bsplines::BSplinePose> initPoseSplineFromCamera(const size_t splineOrder = 6,
																	const size_t poseKnotsPerSecond = 100,
																	const double timeOffsetPadding=0.02) {
	assert(!targetObservation.empty());

    const auto T_c_b = T_extrinsic.T();
    auto rotationVector = boost::make_shared<sm::kinematics::RotationVector>();
    auto pose = boost::make_shared<bsplines::BSplinePose>(splineOrder, rotationVector);

    // Get the checkerboard times.
    std::vector<double> timesVec;
    timesVec.reserve(targetObservations.size());
    std::vector<Eigen::VectorXd> curveVec;
    curveVec.reserve(targetObservations.size());
    for (int idx = 0 ; idx < targetObservations.size() ; ++idx) {
      const auto targetObs = targetObservations[static_cast<size_t>(idx)];
      timesVec.push_back(toSec(targetObs.time) + timeshiftCamToImuPrior);
      const auto trans = targetObs.T_t_c.T() * T_c_b;
      const auto column = pose->transformationToCurveValue(trans);
      curveVec.push_back(column);
    }

	Eigen::VectorXd times(targetObservations.size()+2);
	Eigen::Matrix<double, 6, Eigen::Dynamic> curve(6, targetObservations.size()+2);

	auto isNan = [](const Eigen::MatrixXd& mat) {
	  return (mat.array() == mat.array()).all();
	};

	if (isNan(curve)) {
	  throw std::runtime_error("NaNs in curve values");
	}

	// Add 2 seconds on either end to allow the spline to slide during optimization
	times[0] = timesVec.front() - (timeOffsetPadding * 2.0);
	curve.block(0,0,6,1) = curveVec.front();
	times[static_cast<int>(targetObservations.size()+1)]  =timesVec.back() + (timeOffsetPadding*2.0);
	curve.block(0,static_cast<int>(targetObservations.size()+1), 6,1) = curveVec.back();
	for (int idx = 0 ; idx < targetObservations.size() ; ++idx) {
	  times[idx+1] = timesVec[static_cast<size_t>(idx)];
	  curve.block(0, idx+1, 6,1) = curveVec[static_cast<size_t>(idx)];
	}

	// Make sure the rotation vector doesn't flip
	for (int i = 1 ; i < curve.cols() ; ++i) {
	  const auto previousRotationVector = curve.block(3,i-1, 6,1);
	  const auto r = curve.block(3,i, 6,1);
	  const auto angle = r.norm();
	  const auto axis = r / angle;
	  
	  auto best_r = r;
	  auto best_dist = (best_r - previousRotationVector).norm();
	  for (int s = -3 ; s <= 3 ; ++ s) {
	    const auto aa = axis * (angle + M_PI * 2.0 * s);
	    const auto dist = (aa - previousRotationVector).norm();
	    if (dist < best_dist) {
	      best_r = aa;
	      best_dist = dist;
	    }
	  }
	  curve.block(3,i,6,1) = best_r;
	}
	
	const auto seconds = timesVec.back() - timesVec.front();
	const auto knots = static_cast<int>(std::round(seconds * poseKnotsPerSecond));
	
	std::cout << "Initializing a pose spline with " << knots
	<< " knots (" << poseKnotsPerSecond
	<< " knots per second over " << seconds << " seconds)" << std::endl;
    pose->initPoseSplineSparse(times, curve, knots, 1e-4);
	return pose;
  }

};


struct ImuMeasurement {
  Eigen::Vector3d omega, alpha;
  Eigen::Matrix3d omegaR, omegaInvR, alphaR, alphaInvR;
  double stamp;

  explicit ImuMeasurement(const double _stamp,
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


class IccImu {

public:
  // TODO(radam): ImuParameters?

  // TODO(radam): get imu config

  // TODO(radam): update imu config

protected:

  ImuParameters imuParameters;
  double accelUncertaintyDiscrete;
  double accelRandomWalk;
  double accelUncertainty;
  double gyroUncertaintyDiscrete;
  double gyroRandomWalk;
  double gyroUncertainty;

  Eigen::Vector3d gyroBiasPrior;
  size_t gyroBiasPriorCount = 0;

  Eigen::Vector4d q_i_b_prior;
  boost::shared_ptr<aslam::backend::RotationQuaternion> q_i_b_Dv = nullptr;
  boost::shared_ptr<aslam::backend::EuclideanPoint> r_b_Dv = nullptr;

  double timeOffset = 0.0;

  boost::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> gyroBiasDv = nullptr;
  boost::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> accelBiasDv = nullptr;

  // Error terms
  std::vector<boost::shared_ptr<kalibr_errorterms::EuclideanError>> accelErrors;
  std::vector<boost::shared_ptr<kalibr_errorterms::EuclideanError>> gyroErrors;

public:

   double getAccelUncertaintyDiscrete();

  double getGyroUncertaintyDiscrete();


  IccImu(const ImuParameters& imuParams);


  void addDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem);

  void addAccelerometerErrorTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
								  boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseSplineDv,
								  const Eigen::Vector3d& g_w,
								  double mSigma=0.0,
								  double accelNoiseScale=1.0) ;

  void addGyroscopeErrorTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
							  boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseSplineDv,
							  const Eigen::Vector3d& g_w,
							  double mSigma=0.0,
							  double gyroNoiseScale=1.0);


  void initBiasSplines(/*finish*/) {
	// TODO(radam): finish
  } // TODO(radam): move to cpp

  void addBiasMotionTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem) {
    const auto Wgyro = Eigen::Matrix3d::Identity() / (gyroRandomWalk * gyroRandomWalk);
    const auto Waccel = Eigen::Matrix3d::Identity() / (accelRandomWalk * accelRandomWalk);
    const auto gyroBiasMotionErr = aslam::backend::BSplineMotionError<aslam::splines::EuclideanBSplineDesignVariable>(gyroBiasDv.get(), Wgyro,1);
	// problem->addErrorTerm(gyroBiasMotionErr);     // TODO(radam): fix
	const auto accelBiasMotionErr = aslam::backend::BSplineMotionError<aslam::splines::EuclideanBSplineDesignVariable>(accelBiasDv.get(), Waccel,1);
	// problem->addErrorTerm(accelBiasMotionErr); // TODO(radam): fix

  } // TODO(radam): move to cpp

  void getTransformationFromBodyToImu() {
	// TODO(radam): finish
  } // TODO(radam): move to cpp

  void findOrientationPrior() {
    // TODO(radam): finish
  } // TODO(radam): move to cpp

};

