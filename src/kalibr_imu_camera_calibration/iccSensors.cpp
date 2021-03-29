//
// Created by radam on 2021-03-25.
//

#include <kalibr_imu_camera_calibration/iccSensors.hpp>

double toSec(const int64_t time ) {
  return static_cast<double>(time) / 1e6;
}

IccCamera::IccCamera(const double reprojectionSigma,
					 const bool showCorners,
					 const bool showReproj,
					 const bool showOneStep) : cornerUncertainty(reprojectionSigma){

  // targetObservations = false; // TODO(radam): They need to be passed somehow?
  gravity_w = Eigen::Vector3d(9.80655, 0., 0.);

}

boost::shared_ptr<bsplines::BSplinePose> IccCamera::initPoseSplineFromCamera(const size_t splineOrder,
																			 const size_t poseKnotsPerSecond,
																			 const double timeOffsetPadding) {
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
