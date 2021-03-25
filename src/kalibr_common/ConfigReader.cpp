//
// Created by radam on 2021-03-23.
//

#include <kalibr_common/ConfigReader.hpp>

//#include <aslam/cameras/EquidistantDistortion.hpp>

PinholeCamera::PinholeCamera(const std::vector<double> &intrinsics,const std::vector<double> &dist_coeff,const cv::Size &resolution) {

  const auto focalLength = std::vector<double>(intrinsics.begin(), intrinsics.begin()+2);
  const auto principalPoint = std::vector<double>(intrinsics.begin()+2, intrinsics.begin()+4);




}