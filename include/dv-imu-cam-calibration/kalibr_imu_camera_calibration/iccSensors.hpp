//
// Created by radam on 2021-03-26.
//

#pragma once

#include <kalibr_imu_camera_calibration/common.hpp>

#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <kalibr_common/ConfigReader.hpp>
#include <kalibr_errorterms/EuclideanError.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>

#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/backend/BSplineMotionError.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/TransformationBasic.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>

#include <bsplines/BSpline.hpp>

#include <boost/make_shared.hpp>

#include <Eigen/Eigen>

double toSec(const int64_t time);


class IccImu;

class IccCamera {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:



  const double cornerUncertainty;
  sm::kinematics::Transformation T_extrinsic;
  double timeshiftCamToImuPrior = 0.0;
  PinholeEquidistantCamera camera{std::vector<double>{604.5911733980397, 604.2336278279186, 282.3605083440955, 250.5144138417647},
      std::vector<double>{-0.05965984963878861, 0.11156790983914057, -0.397476602431665,
						  0.4856393825761525},
      cv::Size(640, 480)}; // TODO(radam): pass these ALL in constructor

  Eigen::Vector3d gravity_w;
  double timeOffset = 0.0;

  // Design variables
  boost::shared_ptr<aslam::backend::RotationQuaternion> T_c_b_Dv_q = nullptr;
  boost::shared_ptr<aslam::backend::EuclideanPoint> T_c_b_Dv_t = nullptr;
  boost::shared_ptr<aslam::backend::TransformationBasic> T_c_b_Dv = nullptr;
  boost::shared_ptr<aslam::backend::Scalar> cameraTimeToImuTimeDv = nullptr;

  // Observations of the calibration target
  boost::shared_ptr<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>> targetObservations = nullptr;


public:
  IccCamera(boost::shared_ptr<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>> observations,
  		    const double reprojectionSigma=1.0,
			const bool showCorners=true,
			const bool showReproj = true,
			const bool showOneStep=false);


  sm::kinematics::Transformation getTransformation();

  void findOrientationPriorCameraToImu(boost::shared_ptr<IccImu> iccImu);

  boost::shared_ptr<aslam::cameras::CameraGeometryBase> getCameraGeometry() {
    return camera.getGeometry();
  }

  Eigen::Vector3d getEstimatedGravity();

  void findTimeshiftCameraImuPrior(boost::shared_ptr<IccImu> iccImu, bool verbose);


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
						   double timeOffsetPadding=0.0 );

};


struct ImuMeasurement {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
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

  // Imu measurement data
  boost::shared_ptr<std::vector<ImuMeasurement>> imuData = nullptr;

public:

   double getAccelUncertaintyDiscrete();

  double getGyroUncertaintyDiscrete();


  IccImu(const ImuParameters& imuParams, boost::shared_ptr<std::vector<ImuMeasurement>> data);


  std::vector<ImuMeasurement>& getImuData();



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

  void addBiasMotionTerms(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem) ;

};

