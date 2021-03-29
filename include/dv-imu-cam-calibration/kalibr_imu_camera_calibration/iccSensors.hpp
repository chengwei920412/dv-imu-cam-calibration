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
																	const double timeOffsetPadding=0.02);

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

