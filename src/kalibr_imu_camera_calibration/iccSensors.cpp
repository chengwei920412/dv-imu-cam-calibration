//
// Created by radam on 2021-03-25.
//

#include <kalibr_imu_camera_calibration/iccSensors.hpp>

IccCamera::IccCamera(const double reprojectionSigma,
					 const bool showCorners,
					 const bool showReproj,
					 const bool showOneStep) : cornerUncertainty(reprojectionSigma){

  targetObservations = false; // TODO(radam): pass these from constructor?
  gravity_w = Eigen::Vector3d(9.80655, 0., 0.);

}

double IccImu::getAccelUncertaintyDiscrete() {
  return accelUncertaintyDiscrete;
}

double IccImu::getGyroUncertaintyDiscrete() {
  return gyroUncertaintyDiscrete;
}

IccImu::IccImu(const ImuParameters &imuParams)  :imuParameters(imuParams) {

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

}

void IccImu::addDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem) {
  //gyroBiasDv = aslam::splines::EuclideanBSplineDesignVariable(gyroBias); // TODO(radam): fix
  //accelBiasDv = aslam::splines::EuclideanBSplineDesignVariable(accelBias); // TODO(radam): fix

  // TODO(radam): implement the rest

}

void IccImu::addAccelerometerErrorTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
										boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseSplineDv,
										const Eigen::Vector3d &g_w,
										double mSigma,
										double accelNoiseScale) {

  const double weight = 1.0 / accelNoiseScale;

  size_t numSkipped = 0;

  boost::shared_ptr<aslam::backend::MEstimator> mest;
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
                             w_dot_b.cross(r_b) + w_b.cross(w_b.cross(r_b)));
	  auto aerr = boost::make_shared<kalibr_errorterms::EuclideanError>(im.alpha, im.alphaInvR * weight, a + b_i);
	  aerr->setMEstimatorPolicy(mest);
	  accelErrors.push_back(aerr);
	  problem->addErrorTerm(aerr);
	} else {
	  ++numSkipped;
	}
  }

  std::cout << "Added " << imuData.size() - numSkipped << " of " << imuData.size() << " accelerometer error terms "
			<< "(skipped " << numSkipped << " out-of-bounds measurements)" << std::endl;
}

void IccImu::addGyroscopeErrorTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
									boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseSplineDv,
									const Eigen::Vector3d &g_w,
									double mSigma,
									double gyroNoiseScale) {
  const double weight = 1.0 / gyroNoiseScale;

  size_t numSkipped = 0;

  boost::shared_ptr<aslam::backend::MEstimator> mest;
  if (mSigma > 0.0) {
	mest =  std::make_unique<aslam::backend::HuberMEstimator>(mSigma);
  } else {
	mest = std::make_unique<aslam::backend::NoMEstimator>();
  }

  std::vector<ImuMeasurement> imuData; // TODO(radam): this has to be passed somehow
  for (const auto& im : imuData) {
	const auto tk = im.stamp + timeOffset;
	if (tk > poseSplineDv->spline().t_min() && tk < poseSplineDv->spline().t_max()) {
	  const auto w_b = poseSplineDv->angularVelocityBodyFrame(tk);
	  const auto b_i = gyroBiasDv->toEuclideanExpression(tk,0);
	  const auto C_i_b =q_i_b_Dv->toExpression();
	  const auto w = C_i_b * w_b;
	  auto gerr = boost::make_shared<kalibr_errorterms::EuclideanError>(im.omega, im.omegaInvR * weight, w + b_i);
	  gerr->setMEstimatorPolicy(mest);
	  gyroErrors.push_back(gerr);
	  problem->addErrorTerm(gerr);
	} else {
	  ++numSkipped;
	}

  }

  std::cout << "Added " << imuData.size() - numSkipped << " of " << imuData.size() << " gyroscope error terms "
			<< "(skipped " << numSkipped << " out-of-bounds measurements)" << std::endl;

}
