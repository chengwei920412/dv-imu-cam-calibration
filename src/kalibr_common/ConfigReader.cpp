//
// Created by radam on 2021-03-23.
//

#include <kalibr_common/ConfigReader.hpp>


#include <aslam/cameras.hpp>

PinholeEquidistantCamera::PinholeEquidistantCamera(const std::vector<double> &intrinsics, const std::vector<double> &distCoeff, const cv::Size &resolution) {


  assert(intrinsics.size() >= 4);
  const auto focalLength = std::vector<double>(intrinsics.begin(), intrinsics.begin()+2);
  const auto principalPoint = std::vector<double>(intrinsics.begin()+2, intrinsics.begin()+4);

  assert(distCoeff.size() >= 4);
  const auto dist = aslam::cameras::EquidistantDistortion(distCoeff.at(0), distCoeff.at(1), distCoeff.at(2), distCoeff.at(3));
  const auto proj = aslam::cameras::PinholeProjection<aslam::cameras::EquidistantDistortion>(focalLength[0],
																							 focalLength[1],
																							 principalPoint[0],
																							 principalPoint[1],
																							 resolution.width,
																							 resolution.height,
																							 dist);

  geometry =  boost::make_shared<aslam::cameras::EquidistantDistortedPinholeCameraGeometry>(proj);


}

boost::shared_ptr<aslam::cameras::EquidistantDistortedPinholeCameraGeometry> PinholeEquidistantCamera::getGeometry() {
  return geometry;
}

boost::shared_ptr<aslam::Frame<aslam::cameras::EquidistantDistortedPinholeCameraGeometry>> PinholeEquidistantCamera::frame() {
  auto frame = boost::make_shared<aslam::Frame<aslam::cameras::EquidistantDistortedPinholeCameraGeometry>>(aslam::Frame<aslam::cameras::EquidistantDistortedPinholeCameraGeometry>());
  return frame;
}