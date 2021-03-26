//
// Created by radam on 2021-03-26.
//

#pragma once

#include <sm/kinematics/Transformation.hpp>
#include <kalibr_common/ConfigReader.hpp>
#include <kalibr_errorterms/EuclideanError.hpp>

#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/backend/BSplineMotionError.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanPoint.hpp>

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
  double timeOffset = 0.0;



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
  boost::shared_ptr<aslam::backend::RotationQuaternion> q_i_b_Dv = nullptr;
  boost::shared_ptr<aslam::backend::EuclideanPoint> r_b_Dv = nullptr;

  double timeOffset = 0.0;

  boost::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> gyroBiasDv = nullptr;
  boost::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> accelBiasDv = nullptr;

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

  void addDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem) {
    //gyroBiasDv = aslam::splines::EuclideanBSplineDesignVariable(gyroBias); // TODO(radam): fix
    //accelBiasDv = aslam::splines::EuclideanBSplineDesignVariable(accelBias); // TODO(radam): fix

    // TODO(radam): implement the rest


  } // TODO(radam): move to cpp

  void addAccelerometerErrorTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
								  boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseSplineDv,
								  const Eigen::Vector3d& g_w,
								  double mSigma=0.0,
								  double accelNoiseScale=1.0) {

     const double weight = 1.0 / accelNoiseScale;
     std::vector<double> accelErrors;
     size_t num_skipped = 0;

     std::unique_ptr<aslam::backend::MEstimator> mest;
     if (mSigma > 0.0) {
       mest =  std::make_unique<aslam::backend::HuberMEstimator>(mSigma);
     } else {
       mest = std::make_unique<aslam::backend::NoMEstimator>();
     }

     std::vector<ImuMeasurement> imuData; // TODO(radam): this has to be passed somehow
     for (const auto& im : imuData) {
       const auto tk = im.stamp + timeOffset;
       if (tk > poseSplineDv->spline().t_min() && tk < poseSplineDv->spline().t_max()) {
		 const auto C_b_w = poseSplineDv->orientation(tk).inverse();
		 const auto a_w = poseSplineDv->linearAcceleration(tk);
		 const auto b_i = accelBiasDv->toEuclideanExpression(tk,0);
		 const auto w_b = poseSplineDv->angularVelocityBodyFrame(tk);
		 const auto w_dot_b = poseSplineDv->angularAccelerationBodyFrame(tk);
		 const auto C_i_b = q_i_b_Dv->toExpression();
		 const auto r_b = r_b_Dv->toExpression();
		 const auto a = C_i_b * (C_b_w * (a_w - g_w) + \
                             w_dot_b.cross(r_b) + w_b.cross(w_b.cross(r_b))); finish here
//		 const auto aerr = aslam::backend ket.EuclideanError(im.alpha, im.alphaInvR * weight, a + b_i)
//		 aerr.setMEstimatorPolicy(mest)
//		 accelErrors.append(aerr)
//		 problem.addErrorTerm(aerr)
       } else {
         ++num_skipped;
       }

     }




     // here would be a good next task
    // TODO(radam): finish
  } // TODO(radam): move to cpp

  void addGyroscopeErrorTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem /* finish */, double mSigma=0.0, double gyroNoiseScale=1.0 /* finish*/) {

	// TODO(radam): finish
  } // TODO(radam): move to cpp


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

