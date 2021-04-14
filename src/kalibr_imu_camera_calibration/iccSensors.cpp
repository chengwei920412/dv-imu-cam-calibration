//
// Created by radam on 2021-03-25.
//

#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/ReprojectionError.hpp>
#include <aslam/backend/SimpleReprojectionError.hpp>
#include <aslam/Keypoint.hpp>

#include <kalibr_imu_camera_calibration/iccSensors.hpp>
#include <kalibr_errorterms/GyroscopeError.hpp>

#include <sm/kinematics/transformations.hpp>

double toSec(const int64_t time ) {
  return static_cast<double>(time) / 1e6;
}

IccCamera::IccCamera(boost::shared_ptr<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>> observations,
	 			     const double reprojectionSigma,
					 const bool showCorners,
					 const bool showReproj,
					 const bool showOneStep) : cornerUncertainty(reprojectionSigma){

  targetObservations = observations;
  gravity_w = Eigen::Vector3d(9.80655, 0., 0.);
  T_extrinsic = sm::kinematics::Transformation();
}

sm::kinematics::Transformation IccCamera::getTransformation() {
  assert(T_c_b_Dv_q != nullptr);
  assert(T_c_b_Dv_t != nullptr);
  assert(T_c_b_Dv != nullptr);

  return sm::kinematics::Transformation(T_c_b_Dv_q->getQuaternion(), T_c_b_Dv_t->toEuclidean());
}

void IccCamera::findOrientationPriorCameraToImu(boost::shared_ptr<IccImu> iccImu) {
  std::cout << std::endl << "Estimating imu-camera rotation prior" << std::endl;

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
	  const auto omega_predicted = R_i_c * aslam::backend::EuclideanExpression(poseSpline->angularVelocityBodyFrame(tk).transpose());
	  const auto omega_measured = im.omega;

      // error term
      auto gerr = boost::make_shared<kalibr_errorterms::GyroscopeError>(omega_measured, im.omegaInvR, omega_predicted, bias);
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
    optimizer.optimize();
  } catch (...) {
    throw std::runtime_error("Failed to obtain orientation prior!");
  }

  // overwrite the external rotation prior (keep the external translation prior)
  const auto R_i_c = q_i_c_Dv->toRotationMatrix().transpose();
  T_extrinsic = sm::kinematics::Transformation( sm::kinematics::rt2Transform( R_i_c, T_extrinsic.t() ) );

  // estimate gravity in the world coordinate frame as the mean specific force
  std::vector<Eigen::Vector3d> a_w;
  for (const auto& im : iccImu->getImuData()) {
	const auto tk = im.stamp;
	if (tk > poseSpline->t_min() && tk < poseSpline->t_max()) {
	  const auto val = poseSpline->orientation(tk) * R_i_c * im.alpha;
	  a_w.emplace_back(val);
	}
  }

  assert(!a_w.empty());
  const auto mean_a_w = std::accumulate(a_w.begin(), a_w.end(), Eigen::Vector3d(0.,0.,0.)) / a_w.size();
  gravity_w = mean_a_w / mean_a_w.norm() * 9.80655;
  std::cout << "Gravity was intialized to " <<  gravity_w.transpose() <<  " [m/s^2]" << std::endl;

  // set the gyro bias prior (if we have more than 1 cameras use recursive average)
  const auto b_gyro = gyroBiasDv->toExpression().toEuclidean();
  iccImu->gyroBiasPriorCount++;
  iccImu->gyroBiasPrior = (iccImu->gyroBiasPriorCount-1.0) /
  	iccImu->gyroBiasPriorCount * iccImu->gyroBiasPrior + 1.0/iccImu->gyroBiasPriorCount*b_gyro;

  // print result
  std::cout << " Transformation prior camera-imu found as: (T_extrinsic)" << std::endl;
  std::cout << T_extrinsic.T() << std::endl;
  std::cout << "  Gyro bias prior found as: (b_gyro)" << std::endl;
  std::cout << b_gyro.transpose() << std::endl;
}

Eigen::Vector3d IccCamera::getEstimatedGravity() {
  return gravity_w;
}

boost::shared_ptr<bsplines::BSplinePose> IccCamera::initPoseSplineFromCamera(const size_t splineOrder,
																			 const size_t poseKnotsPerSecond,
																			 const double timeOffsetPadding) {


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

  Eigen::VectorXd times(targetObservations->size()+2);
  times.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> curve(6, targetObservations->size()+2);
  curve.setZero();
  
  auto isNan = [](const Eigen::MatrixXd& mat) {
	return !(mat.array() == mat.array()).all();
  };

  // Add 2 seconds on either end to allow the spline to slide during optimization
  times[0] = timesVec.front() - (timeOffsetPadding * 2.0);
  curve.block(0,0,6,1) = curveVec.front();
  times[static_cast<int>(targetObservations->size()+1)]  =timesVec.back() + (timeOffsetPadding*2.0);
  curve.block(0,static_cast<int>(targetObservations->size()+1), 6,1) = curveVec.back();
  for (int idx = 0 ; idx < targetObservations->size() ; ++idx) {
	times[idx+1] = timesVec[static_cast<size_t>(idx)];
	curve.block(0, idx+1, 6,1) = curveVec[static_cast<size_t>(idx)];
  }

  if (isNan(curve)) {
	throw std::runtime_error("NaNs in curve values");
  }
  
  // Make sure the rotation vector doesn't flip
  for (int i = 1 ; i < curve.cols() ; ++i) {
	const auto previousRotationVector = curve.block(3,i-1, 3,1);
	const auto r = curve.block(3,i, 3,1);
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
	curve.block(3,i,3,1) = best_r;
  }
  

  const auto seconds = timesVec.back() - timesVec.front();
  const auto knots = static_cast<int>(std::round(seconds * poseKnotsPerSecond));

  std::cout << "Initializing a pose spline with " << knots
			<< " knots (" << poseKnotsPerSecond
			<< " knots per second over " << seconds << " seconds)" << std::endl;
  pose->initPoseSplineSparse(times, curve, knots, 1e-4);
  return pose;
}

void IccCamera::addDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
								   bool noExtrinsics,
								   bool noTimeCalibration,
								   size_t baselinedv_group_id) {
  T_c_b_Dv_q = boost::make_shared<aslam::backend::RotationQuaternion>(T_extrinsic.q());
  T_c_b_Dv_q->setActive(true);
  problem->addDesignVariable(T_c_b_Dv_q, baselinedv_group_id);

  T_c_b_Dv_t = boost::make_shared<aslam::backend::EuclideanPoint>(T_extrinsic.t());
  T_c_b_Dv_t->setActive(true);
  problem->addDesignVariable(T_c_b_Dv_t, baselinedv_group_id);

  T_c_b_Dv = boost::make_shared<aslam::backend::TransformationBasic>(T_c_b_Dv_q->toExpression(), T_c_b_Dv_t->toExpression());

  cameraTimeToImuTimeDv = boost::make_shared<aslam::backend::Scalar>(0.0);
  cameraTimeToImuTimeDv->setActive(!noTimeCalibration);
  problem->addDesignVariable(cameraTimeToImuTimeDv, CALIBRATION_GROUP_ID);
}

void IccCamera::addCameraErrorTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
									boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseSplineDv,
									double blakeZissermanDf,
									double timeOffsetPadding) {
  std::cout << "Adding camera error terms (" << targetObservations->size() << ") ..." << std::endl;

  const auto T_cN_b = T_c_b_Dv->toExpression();

  for (const auto& [ts, obs] : *targetObservations) {
	const auto frameTime = cameraTimeToImuTimeDv->toExpression() + obs.time().toSec() + timeshiftCamToImuPrior;
	const auto frameTimeScalar = frameTime.toScalar();

	// #as we are applying an initial time shift outside the optimization so
	// #we need to make sure that we dont add data outside the spline definition
	if (frameTimeScalar <= poseSplineDv->spline().t_min() || frameTimeScalar >= poseSplineDv->spline().t_max()) {
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
	  R(0,0) = cornerUncVal;
	  R(1,1) = cornerUncVal;
	  const auto invR = R.inverse();


	  for (size_t idx = 0 ; idx < imageCornerPoints.size() ; ++idx) {
		const auto &imageCornerPoint = imageCornerPoints[idx];

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


	  for (size_t idx = 0 ; idx < imageCornerPoints.size() ; ++idx) {
	    const auto& imageCornerPoint = imageCornerPoints[idx];
	    const auto& targetPoint = targetCornerPoints[idx];

	    // Target points
	    const Eigen::Vector3d eigenPoint(static_cast<double>(targetPoint.x), static_cast<double>(targetPoint.y), static_cast<double>(targetPoint.z));
		const auto p = T_c_w *  aslam::backend::HomogeneousExpression(eigenPoint);

		// #build and append the error term
		auto rerr = boost::make_shared<aslam::backend::SimpleReprojectionError<aslam::Frame<aslam::cameras::EquidistantDistortedPinholeCameraGeometry>>>(frame.get(), idx, p);
		
		// #add blake-zisserman m-estimator
		if (blakeZissermanDf > 0.0) {
       	  throw std::runtime_error("Not implemented blake-zisserman");
		  // mest = aopt.BlakeZissermanMEstimator( blakeZissermanDf )
		  // rerr.setMEstimatorPolicy(mest)
		}

		problem->addErrorTerm(rerr);
	  }
  }

  std::cout << "  Added " << targetObservations->size() << " camera error tems" << std::endl;


}

double IccImu::getAccelUncertaintyDiscrete() {
  return accelUncertaintyDiscrete;
}

double IccImu::getGyroUncertaintyDiscrete() {
  return gyroUncertaintyDiscrete;
}

IccImu::IccImu(const ImuParameters &imuParams, boost::shared_ptr<std::vector<ImuMeasurement>> data)  :imuParameters(imuParams) {
  imuData = data;

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

std::vector<ImuMeasurement>& IccImu::getImuData() {
  return *imuData;
}

void IccImu::addDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem) {
  gyroBiasDv = boost::make_shared<aslam::splines::EuclideanBSplineDesignVariable>(*gyroBias);
  accelBiasDv = boost::make_shared<aslam::splines::EuclideanBSplineDesignVariable>(*accelBias);

  addSplineDesignVariables(problem, gyroBiasDv, true, HELPER_GROUP_ID);
  addSplineDesignVariables(problem, accelBiasDv, true, HELPER_GROUP_ID);

  q_i_b_Dv = boost::make_shared<aslam::backend::RotationQuaternion>(q_i_b_prior);
  problem->addDesignVariable(q_i_b_Dv, HELPER_GROUP_ID);
  q_i_b_Dv->setActive(true);
  r_b_Dv = boost::make_shared<aslam::backend::EuclideanPoint>(Eigen::Vector3d(0.0, 0.0, 0.0));
  problem->addDesignVariable(r_b_Dv, HELPER_GROUP_ID);
  r_b_Dv->setActive(true);
}

void IccImu::addAccelerometerErrorTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
										boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseSplineDv,
										const Eigen::Vector3d &g_w,
										double mSigma,
										double accelNoiseScale) {
  std::cout << "Adding accelerometer error terms (" << imuData->size() << ") ..." << std::endl;


  const double weight = 1.0 / accelNoiseScale;

  size_t numSkipped = 0;

  boost::shared_ptr<aslam::backend::MEstimator> mest;
  if (mSigma > 0.0) {
	mest =  std::make_unique<aslam::backend::HuberMEstimator>(mSigma);
  } else {
	mest = std::make_unique<aslam::backend::NoMEstimator>();
  }

  for (const auto& im : *imuData) {
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

  std::cout << "  Added " << imuData->size() - numSkipped << " of " << imuData->size() << " accelerometer error terms "
			<< "(skipped " << numSkipped << " out-of-bounds measurements)" << std::endl;
}

void IccImu::addGyroscopeErrorTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
									boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseSplineDv,
									const Eigen::Vector3d &g_w,
									double mSigma,
									double gyroNoiseScale) {
  std::cout << "Adding gyroscope error terms (" << imuData->size() << ") ..." << std::endl;

  const double weight = 1.0 / gyroNoiseScale;

  size_t numSkipped = 0;

  boost::shared_ptr<aslam::backend::MEstimator> mest;
  if (mSigma > 0.0) {
	mest =  std::make_unique<aslam::backend::HuberMEstimator>(mSigma);
  } else {
	mest = std::make_unique<aslam::backend::NoMEstimator>();
  }

  for (const auto& im : *imuData) {
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

  std::cout << "  Added " << imuData->size() - numSkipped << " of " << imuData->size() << " gyroscope error terms "
			<< "(skipped " << numSkipped << " out-of-bounds measurements)" << std::endl;

}

void IccImu::initBiasSplines(boost::shared_ptr<bsplines::BSplinePose> poseSpline,
							 const size_t splineOrder,
							 const size_t biasKnotsPerSecond) {
  const auto start = poseSpline->t_min();
  const auto end = poseSpline->t_max();
  const auto seconds = end - start;
  const auto knots = static_cast<size_t>(round(seconds * static_cast<double>(biasKnotsPerSecond)));

  std::cout << "Initializing the bias splines with " << knots << " knots" << std::endl;

  // initialize the bias splines
  gyroBias = boost::make_shared<bsplines::BSpline>(splineOrder);
  gyroBias->initConstantSpline(start,end,knots, gyroBiasPrior);

  accelBias = boost::make_shared<bsplines::BSpline>(splineOrder);
  accelBias->initConstantSpline(start,end,knots, Eigen::Vector3d(0.,0.,0.));
}

void IccImu::addBiasMotionTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem) {
  const auto Wgyro = Eigen::Matrix3d::Identity() / (gyroRandomWalk * gyroRandomWalk);
  const auto Waccel = Eigen::Matrix3d::Identity() / (accelRandomWalk * accelRandomWalk);
  const auto gyroBiasMotionErr = boost::make_shared<aslam::backend::BSplineMotionError<aslam::splines::EuclideanBSplineDesignVariable>>(gyroBiasDv.get(), Wgyro,1);
  problem->addErrorTerm(gyroBiasMotionErr);
  const auto accelBiasMotionErr = boost::make_shared<aslam::backend::BSplineMotionError<aslam::splines::EuclideanBSplineDesignVariable>>(accelBiasDv.get(), Waccel,1);
  problem->addErrorTerm(accelBiasMotionErr);
}