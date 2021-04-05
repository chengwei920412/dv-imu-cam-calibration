//
// Created by radam on 2021-03-23.
//

#pragma once

#include <aslam/cameras.hpp>
#include <aslam/cameras/EquidistantDistortion.hpp>
#include <aslam/cameras/PinholeProjection.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/Frame.hpp>

#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>

#include <vector>

class PinholeEquidistantCamera {

public:

protected:

  boost::shared_ptr<aslam::cameras::EquidistantDistortedPinholeCameraGeometry> geometry = nullptr;

public:

  PinholeEquidistantCamera(const std::vector<double>& intrinsics, const std::vector<double>& distCoeff, const cv::Size& resolution);

  static PinholeEquidistantCamera fromParameters() {}; // TODO(radam): implement?

  boost::shared_ptr<aslam::cameras::EquidistantDistortedPinholeCameraGeometry> getGeometry();

  boost::shared_ptr<aslam::Frame<aslam::cameras::EquidistantDistortedPinholeCameraGeometry>> frame() ;


};


struct CameraParameters {

  std::vector<double> intrinsics{604.5911733980397, 604.2336278279186, 282.3605083440955, 250.5144138417647};
  std::vector<double> distCoeff{-0.05965984963878861, 0.11156790983914057, -0.397476602431665, 0.4856393825761525};
  cv::Size resolution{640, 480};



};

struct ImuParameters {
  double updateRate = 200.0;
  double accNoiseDensity = 1.4e-3;
  double accRandomWalk = 8.6e-5;
  double gyrNoiseDensity = 8.0e-5;
  double gytRandomWalk = 2.2e-6;

  std::tuple<double, double, double> getAccelerometerStatistics() const {

    double accelUncertaintyDiscrete = accNoiseDensity / sqrt(1.0 / updateRate);
	return std::make_tuple(accelUncertaintyDiscrete,  accRandomWalk, accNoiseDensity);
  }

  std::tuple<double, double, double> getGyroStatistics() const {
	double gyroUncertaintyDiscrete = gyrNoiseDensity / sqrt(1.0 / updateRate);
	return std::make_tuple(gyroUncertaintyDiscrete, gytRandomWalk, gyrNoiseDensity);
  }



};

struct AsymmetricCircleGridParameters {
  std::string targetType{"circlegrid"};
  size_t targetRows = 4;
  size_t targetCols = 11;
  double spacingMeters = 0.05;

};

// TODO(radam): camera chain skipped as we only want one camera