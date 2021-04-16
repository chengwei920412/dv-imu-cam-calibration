//
// Created by radam on 2021-03-23.
//

#pragma once

#include <aslam/backend/CameraDesignVariable.hpp>

#include <aslam/cameras.hpp>
#include <aslam/cameras/EquidistantDistortion.hpp>
#include <aslam/cameras/PinholeProjection.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/Frame.hpp>

#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>

#include <vector>


class PinholeRadialTangentialCamera {

protected:

  boost::shared_ptr<aslam::cameras::DistortedPinholeCameraGeometry> geometry = nullptr;

  std::vector<double> focalLength;
  std::vector<double> principalPoint;
  std::vector<double> distortionCoefficients;

public:

  PinholeRadialTangentialCamera(const std::vector<double>& intrinsics,
								const std::vector<double>& distCoeff,
								const cv::Size& resolution);


  boost::shared_ptr<aslam::cameras::DistortedPinholeCameraGeometry> getGeometry();

  boost::shared_ptr<aslam::Frame<aslam::cameras::DistortedPinholeCameraGeometry>> frame() ;

  void printDetails();


};


struct ImuParameters {
  double updateRate = 200.0;
  double accNoiseDensity = 1.4e-3;
  double accRandomWalk = 8.6e-5;
  double gyrNoiseDensity = 8.0e-5;
  double gyrRandomWalk = 2.2e-6;

  std::tuple<double, double, double> getAccelerometerStatistics() const {

    double accelUncertaintyDiscrete = accNoiseDensity / sqrt(1.0 / updateRate);
	return std::make_tuple(accelUncertaintyDiscrete,  accRandomWalk, accNoiseDensity);
  }

  std::tuple<double, double, double> getGyroStatistics() const {
	double gyroUncertaintyDiscrete = gyrNoiseDensity / sqrt(1.0 / updateRate);
	return std::make_tuple(gyroUncertaintyDiscrete, gyrRandomWalk, gyrNoiseDensity);
  }



};
