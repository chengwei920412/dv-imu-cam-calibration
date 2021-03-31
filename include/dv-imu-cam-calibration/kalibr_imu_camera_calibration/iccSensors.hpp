//
// Created by radam on 2021-03-26.
//

#pragma once

#include <kalibr_imu_camera_calibration/common.hpp>

#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <kalibr_common/ConfigReader.hpp>
#include <kalibr_errorterms/EuclideanError.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>

#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/backend/BSplineMotionError.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/TransformationBasic.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/OptimizationProblem.hpp>

#include <bsplines/BSpline.hpp>

#include <boost/make_shared.hpp>

#include <Eigen/Eigen>

double toSec(const int64_t time);

struct TargetObservation {
  int64_t time;
  sm::kinematics::Transformation T_t_c;
  std::vector<cv::Point> cornersTargetFrame; // TODO(radam): not sure if this should be a variable member method that does some computations
  std::vector<cv::Point> cornersImageFrame;
};

class IccImu;

class IccCamera {

protected:
  // TODO(radam): dataset
  // TODO(radam): cam config
  // TODO(radam): target config

  //        distortion_coefficients: [],
  //        distortion_type: radialtangential,
  //        focal_length: [],
  //        principal_point: []}

  const double cornerUncertainty;
  sm::kinematics::Transformation T_extrinsic;
  double timeshiftCamToImuPrior = 0.0;
  PinholeEquidistantCamera camera{std::vector<double>{6.1982681497359658e+02, 6.2027527379491937e+02, 2.9590645781122009e+02, 2.4359719473954027e+02},
      std::vector<double>{-3.8620817947560659e-01, 2.7670984121686787e-01, -1.2984654836296217e-04, -6.9927656141977837e-04, -2.0510314474330094e-01},
      cv::Size(640, 480)}; // TODO(radam): pass these in constructor
  std::vector<TargetObservation> targetObservations;
  Eigen::Vector3d gravity_w;
  double timeOffset = 0.0;

  // Design variables
  boost::shared_ptr<aslam::backend::RotationQuaternion> T_c_b_Dv_q = nullptr;
  boost::shared_ptr<aslam::backend::EuclideanPoint> T_c_b_Dv_t = nullptr;
  boost::shared_ptr<aslam::backend::TransformationBasic> T_c_b_Dv = nullptr;
  boost::shared_ptr<aslam::backend::Scalar> cameraTimeToImuTimeDv = nullptr;



public:
  IccCamera(const double reprojectionSigma=1.0,
			const bool showCorners=true,
			const bool showReproj = true,
			const bool showOneStep=false); // TODO(radam): params

  // void setupCalibrationTarget // TODO(radam): make sure it is not needed

  void findOrientationPriorCameraToImu(boost::shared_ptr<IccImu> iccImu);

  boost::shared_ptr<aslam::cameras::CameraGeometryBase> getCameraGeometry() {
    return camera.getGeometry();
  }

  Eigen::Vector3d getEstimatedGravity();

  // TODO(radam): missing methods here

  boost::shared_ptr<bsplines::BSplinePose> initPoseSplineFromCamera(const size_t splineOrder = 6,
																	const size_t poseKnotsPerSecond = 100,
																	const double timeOffsetPadding=0.02);

  void addDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
						  bool noExtrinsics = true,
						  bool noTimeCalibration = true,
						  size_t baselinedv_group_id = HELPER_GROUP_ID);

  void addCameraErrorTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
						   boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseSplineDv,
						   double blakeZissermanDf=0.0,
						   double timeOffsetPadding=0.0 ) {
    const auto T_cN_b = T_c_b_Dv->toExpression();

    std::vector<double> allReprojectionErrors; // TODO(radam): this should be a member variable
    allReprojectionErrors.clear();

    for (const auto& obs : targetObservations) {
      const auto frameTime = cameraTimeToImuTimeDv->toExpression() + toSec(obs.time) + timeshiftCamToImuPrior;
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
	  //imageCornerPoints =  np.array( obs.getCornersImageFrame() ).T // TODO(radam): detections need to be finished
	  //targetCornerPoints = np.array( obs.getCornersTargetFrame() ).T

	  // TODO(radam): finish here
//	  // #setup an aslam frame (handles the distortion)
//	  frame = camera.frameType();
//	  frame.setGeometry(self.camera.geometry)
//
//	  // #corner uncertainty
//	  R = np.eye(2) * self.cornerUncertainty * self.cornerUncertainty
//	  invR = np.linalg.inv(R)
//
//	  for pidx in range(0,imageCornerPoints.shape[1]):
//	  // #add all image points
//	  k = self.camera.keypointType()
//	  k.setMeasurement( imageCornerPoints[:,pidx] )
//	  k.setInverseMeasurementCovariance(invR)
//	  frame.addKeypoint(k)
//
//	  reprojectionErrors=list()
//	  for pidx in range(0,imageCornerPoints.shape[1]):
//	  // #add all target points
//	  targetPoint = np.insert( targetCornerPoints.transpose()[pidx], 3, 1)
//	  p = T_c_w *  aopt.HomogeneousExpression( targetPoint )
//
//	  // #build and append the error term
//	  rerr = error_t(frame, pidx, p)
//
//	  // #add blake-zisserman m-estimator
//	  if blakeZissermanDf>0.0:
//	  mest = aopt.BlakeZissermanMEstimator( blakeZissermanDf )
//	  rerr.setMEstimatorPolicy(mest)
//
//	  problem.addErrorTerm(rerr)
//	  reprojectionErrors.append(rerr)
//
//	  allReprojectionErrors.append(reprojectionErrors)
    }

    std::cout << "  Added " << targetObservations.size() << " camera error tems" << std::endl;


  } // TODO(radam): move to cpp

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

  std::vector<ImuMeasurement> imuData; // TODO(radam): this probably should not be public

  Eigen::Vector3d gyroBiasPrior;
  size_t gyroBiasPriorCount = 0;

protected:

  ImuParameters imuParameters;
  double accelUncertaintyDiscrete;
  double accelRandomWalk;
  double accelUncertainty;
  double gyroUncertaintyDiscrete;
  double gyroRandomWalk;
  double gyroUncertainty;



  Eigen::Vector4d q_i_b_prior;
  boost::shared_ptr<aslam::backend::RotationQuaternion> q_i_b_Dv = nullptr;
  boost::shared_ptr<aslam::backend::EuclideanPoint> r_b_Dv = nullptr;

  double timeOffset = 0.0;

  boost::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> gyroBiasDv = nullptr;
  boost::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> accelBiasDv = nullptr;

  // Error terms
  std::vector<boost::shared_ptr<kalibr_errorterms::EuclideanError>> accelErrors;
  std::vector<boost::shared_ptr<kalibr_errorterms::EuclideanError>> gyroErrors;

  // Bias BSplines
  boost::shared_ptr<bsplines::BSpline> gyroBias = nullptr;
  boost::shared_ptr<bsplines::BSpline> accelBias = nullptr;

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


  void initBiasSplines(boost::shared_ptr<bsplines::BSplinePose> poseSpline,
					   size_t splineOrder,
					   size_t biasKnotsPerSecond);

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

