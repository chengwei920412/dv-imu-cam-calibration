//
// Created by radam on 2021-03-23.
//

#include <kalibr_common/ConfigReader.hpp>


#include <aslam/cameras.hpp>


PinholeRadialTangentialCamera::PinholeRadialTangentialCamera(const std::vector<double> &intrinsics,
															 const std::vector<double> &distCoeff,
															 const cv::Size &resolution) {


  assert(intrinsics.size() >= 4);
  const auto focalLength = std::vector<double>(intrinsics.begin(), intrinsics.begin()+2);
  const auto principalPoint = std::vector<double>(intrinsics.begin()+2, intrinsics.begin()+4);

  assert(distCoeff.size() >= 4);
  const auto dist = aslam::cameras::RadialTangentialDistortion(distCoeff.at(0),
															   distCoeff.at(1),
															   distCoeff.at(2),
															   distCoeff.at(3)); // TODO(radam): double check order
  const auto proj = aslam::cameras::PinholeProjection<aslam::cameras::RadialTangentialDistortion>(focalLength[0],
																							 focalLength[1],
																							 principalPoint[0],
																							 principalPoint[1],
																							 resolution.width,
																							 resolution.height,
																							 dist);

  geometry =  boost::make_shared<aslam::cameras::DistortedPinholeCameraGeometry>(proj);

}

boost::shared_ptr<aslam::cameras::DistortedPinholeCameraGeometry> PinholeRadialTangentialCamera::getGeometry() {
  return geometry;
}

boost::shared_ptr<aslam::Frame<aslam::cameras::DistortedPinholeCameraGeometry>> PinholeRadialTangentialCamera::frame() {
  auto frame = boost::make_shared<aslam::Frame<aslam::cameras::DistortedPinholeCameraGeometry>>(aslam::Frame<aslam::cameras::DistortedPinholeCameraGeometry>());
  return frame;
}