//
// Created by radam on 2021-03-26.
//

#pragma once

#include <sm/kinematics/Transformation.hpp>
#include <kalibr_common/ConfigReader.hpp>

#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <aslam/backend/BSplineMotionError.hpp>

#include <Eigen/Eigen>

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
  bool targetObservations; // TODO(radam): type
  Eigen::Vector3d gravity_w;



public:
  IccCamera(const double reprojectionSigma=1.0,
			const bool showCorners=true,
			const bool showReproj = true,
			const bool showOneStep=false); // TODO(radam): params

  // void setupCalibrationTarget // TODO(radam): make sure it is not needed

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
  double timeOffset = 0.0;

  std::unique_ptr<aslam::splines::EuclideanBSplineDesignVariable> gyroBiasDv = nullptr;
  std::unique_ptr<aslam::splines::EuclideanBSplineDesignVariable> accelBiasDv = nullptr;

public:

   double getAccelUncertaintyDiscrete() {
     return accelUncertaintyDiscrete;
   } // TODO(radam): to cpp

  double getGyroUncertaintyDiscrete() {
	return gyroUncertaintyDiscrete;
  } // TODO(radam): to cpp


  IccImu(const ImuParameters& imuParams) :imuParameters(imuParams) {

	const auto [aud, 	arw, 	au ] = imuParams.getAccelerometerStatistics();
	const auto [gud, 	grw, 	gu ] = imuParams.getGyroStatistics();

	accelUncertaintyDiscrete = aud;
	accelRandomWalk = arw;
	accelUncertainty = au;
	gyroUncertaintyDiscrete = gud;
	gyroRandomWalk = grw;
	gyroUncertainty = gu;

	gyroBiasPrior.setZero();

	q_i_b_prior = Eigen::Vector4d(0.,0.,0.,1.);


  } // TODO(radam): move to cpp

  void addDesignVariables(std::unique_ptr<aslam::calibration::OptimizationProblem>& problem) {
    //gyroBiasDv = aslam::splines::EuclideanBSplineDesignVariable(gyroBias); // TODO(radam): fix
    //accelBiasDv = aslam::splines::EuclideanBSplineDesignVariable(accelBias); // TODO(radam): fix

    // TODO(radam): implement the rest


  } // TODO(radam): move to cpp

  void addAccelerometerErrorTerms(std::unique_ptr<aslam::calibration::OptimizationProblem>& problem /* finish */, double mSigma=0.0, double accelNoiseScale=1.0) {

    // TODO(radam): finish
  } // TODO(radam): move to cpp

  void addGyroscopeErrorTerms(std::unique_ptr<aslam::calibration::OptimizationProblem>& problem /* finish */, double mSigma=0.0, double gyroNoiseScale=1.0 /* finish*/) {

	// TODO(radam): finish
  } // TODO(radam): move to cpp


  void initBiasSplines(/*finish*/) {
	// TODO(radam): finish
  } // TODO(radam): move to cpp

  void addBiasMotionTerms(std::unique_ptr<aslam::calibration::OptimizationProblem>& problem) {
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

