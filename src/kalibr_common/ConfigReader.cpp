//
// Created by radam on 2021-03-23.
//

#include <kalibr_common/ConfigReader.hpp>

#include <aslam/cameras/EquidistantDistortion.hpp>

PinholeEquidistantCamera::PinholeEquidistantCamera(const std::vector<double> &intrinsics, const std::vector<double> &distCoeff, const cv::Size &resolution) {

  const auto focalLength = std::vector<double>(intrinsics.begin(), intrinsics.begin()+2);
  const auto principalPoint = std::vector<double>(intrinsics.begin()+2, intrinsics.begin()+4);

  assert(distCoeff.size() >= 4);
  const auto dist = aslam::cameras::EquidistantDistortion(distCoeff.at(0), distCoeff.at(1), distCoeff.at(2), distCoeff.at(3));
  //const auto proj = aslam::cameras:: // TODO(radam): this is missing

  // geometry = // TODO(radam): missing

  // TODO(radam): everything is missing here...



}
