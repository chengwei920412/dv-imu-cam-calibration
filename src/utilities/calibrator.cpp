//
// Created by radam on 2021-03-23.
//


#include <utilities/calibrator.hpp>

#include <sm/boost/JobQueue.hpp>

#include <Eigen/Eigen>

Calibrator::StampedImage previewImageWithText(const std::string &text, const int64_t timestamp = 0LL, const cv::Size &size = cv::Size(640, 480)) {
  cv::Mat img = cv::Mat::zeros(size, CV_8UC3);

  cv::putText(img,
			  text,
			  cv::Point(size.width / 8, size.height / 2),
			  cv::FONT_HERSHEY_DUPLEX,
			  1.0,
			  cv::Scalar(255, 255, 255),
			  2);

  return Calibrator::StampedImage(img, timestamp);
}

Calibrator::Calibrator() : iccImu(imuParameters){
  setPreviewImage(previewImageWithText("No image arrived so far", previewTimestamp));


  detectionsQueue.start(5);


  size_t rows = 11; // TODO(radam): param
  size_t cols = 4; // TODO(radam): param
  double spacingMeters = 0.05; // TODO(radam): param
  calibrationPattern = CalibrationPattern::ASYMMETRIC_CIRCLES_GRID; // TODO(radam): param



  switch (calibrationPattern) {
  case CalibrationPattern::ASYMMETRIC_CIRCLES_GRID: {
	auto options = aslam::cameras::GridCalibrationTargetCirclegrid::CirclegridOptions();
	options.showExtractionVideo = false;
	options.useAsymmetricCirclegrid = true;
	grid = boost::make_shared<aslam::cameras::GridCalibrationTargetCirclegrid>(rows, cols, spacingMeters, options);
    break;
  }
  default:
    throw std::runtime_error("Not implemented calibration pattern");
  }

  detectorOptions.imageStepping = false;
  detectorOptions.plotCornerReprojection = false;
  detectorOptions.filterCornerOutliers = true;
}

void Calibrator::addImu(const int64_t timestamp,
						const double gyroX,
						const double gyroY,
						const double gyroZ,
						const double accelX,
						const double accelY,
						const double accelZ) {

  const double tsS = static_cast<double>(timestamp) / 1e6;
  const auto Rgyro = Eigen::Matrix3d::Identity() * iccImu.getGyroUncertaintyDiscrete();
  const auto Raccel = Eigen::Matrix3d::Identity() * iccImu.getAccelUncertaintyDiscrete();
  const Eigen::Vector3d omega(gyroX, gyroY, gyroZ);
  const Eigen::Vector3d alpha(accelX, accelY, accelZ);
  ImuMeasurement imuMeas(tsS, omega, alpha, Rgyro, Raccel);
  imuData.push_back(imuMeas);
}




void Calibrator::addImage(const StampedImage& stampedImage) {
  boost::unique_future<void> job;
  detectionsQueue.scheduleFuture<void>(boost::bind(&Calibrator::detectPattern, this, stampedImage.clone()), job);
}

void Calibrator::addImage(const cv::Mat &img, int64_t timestamp) {
  addImage(StampedImage(img, timestamp));
}

Calibrator::StampedImage Calibrator::getPreviewImage() {
	std::lock_guard<std::mutex> lock(previewMutex);
	return StampedImage(previewImage, previewTimestamp).clone();
}

void Calibrator::setPreviewImage(const StampedImage &stampedImage) {
  std::lock_guard<std::mutex> lock(previewMutex);
  previewImage = stampedImage.image.clone();
  previewTimestamp = stampedImage.timestamp;
}

void Calibrator::detectPattern(const StampedImage &stampedImage) {

// TODO(radam): read input image and undistort points before passing to kalibr

  std::vector<cv::Point2f> pointBuf;

  // Make a copy of the detector in each thread to avoid memory issues
  auto detector = boost::make_shared<aslam::cameras::GridDetector>(iccCamera.getCameraGeometry(), grid, detectorOptions);

  // Search for pattern and draw it on the image frame
  aslam::cameras::GridCalibrationTargetObservation observation;
  bool success = detector->findTarget(stampedImage.image, aslam::Time(toSec(stampedImage.timestamp)), observation);
  auto preview = stampedImage;

  if (success) {
	if (observation.hasSuccessfulObservation()) {
	  cv::cvtColor(preview.image, preview.image, cv::COLOR_GRAY2BGR);
	  cv::Point prevPoint(-1, -1);
	  for (size_t y = 0; y < grid->rows(); ++y) {
		const auto color = colors[y % colors.size()];
		for (size_t x = 0; x < grid->cols(); ++x) {
		  const auto idx = y * grid->cols() + x;
		  Eigen::Vector2d point;
		  if (observation.imagePoint(idx, point)) {
			const cv::Point cvPoint(point.x(), point.y());
			cv::circle(preview.image, cvPoint, 8, color, 2);
			if (prevPoint != cv::Point(-1, -1)) {
			  cv::line(preview.image, prevPoint, cvPoint, color, 2);
			}
			prevPoint = cvPoint;
		  }
		}
	  }
	  std::lock_guard<std::mutex> lock(targetObservationsMutex);
	  targetObservations.push_back(observation);
	}
  }

  setPreviewImage(preview);
}
