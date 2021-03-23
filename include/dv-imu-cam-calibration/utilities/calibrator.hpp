//
// Created by radam on 2021-03-23.
//

#pragma once

#include <opencv2/opencv.hpp>

class Calibrator {

protected:

public:
  Calibrator();

  cv::Mat process(const cv::Mat& img);

};