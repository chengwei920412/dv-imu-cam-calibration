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

Calibrator::Calibrator() {
  state = INITIALIZED;
  targetObservations = boost::make_shared<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>>();
  iccImu = boost::make_shared<IccImu>(imuParameters);
  iccCamera = boost::make_shared<IccCamera>(targetObservations);
  iccCalibrator = boost::make_shared<IccCalibrator>(iccCamera, iccImu);
  imuData = boost::make_shared<std::vector<ImuMeasurement>>();

  detectionsQueue.start(std::max(1u, std::thread::hardware_concurrency()-1));


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

  iccImu->registerImuData(imuData);
}

void Calibrator::addImu(const int64_t timestamp,
						const double gyroX,
						const double gyroY,
						const double gyroZ,
						const double accelX,
						const double accelY,
						const double accelZ) {
  if (state == INITIALIZED) {
    state = COLLECTING; // TODO(radam): this has to be triggered by DV and data vectors need to be cleaned
  }

  if (state != COLLECTING) {
    return;
  }

  const double tsS = static_cast<double>(timestamp) / 1e6;
  const auto Rgyro = Eigen::Matrix3d::Identity() * iccImu->getGyroUncertaintyDiscrete();
  const auto Raccel = Eigen::Matrix3d::Identity() * iccImu->getAccelUncertaintyDiscrete();
  const Eigen::Vector3d omega(gyroX, gyroY, gyroZ);
  const Eigen::Vector3d alpha(accelX, accelY, accelZ);
  ImuMeasurement imuMeas(tsS, omega, alpha, Rgyro, Raccel);
  imuData->push_back(imuMeas);
}




void Calibrator::addImage(const StampedImage& stampedImage) {
  if (state == INITIALIZED) {
	state = COLLECTING;
  }

  if (state != COLLECTING) {
    return;
  }

  boost::unique_future<void> job;
  detectionsQueue.scheduleFuture<void>(boost::bind(&Calibrator::detectPattern, this, stampedImage.clone()), job);
}

void Calibrator::addImage(const cv::Mat &img, int64_t timestamp) {
  addImage(StampedImage(img, timestamp));
}

Calibrator::StampedImage Calibrator::getPreviewImage() {


  switch (state) {
  case INITIALIZED:
	return StampedImage(previewImageWithText("Waiting for the first image to arrive"));
    break;
  case COLLECTING: {
	std::stringstream ss;

	// Get the latest image
	StampedImage stampedImage;
	{
	  std::lock_guard<std::mutex> lock(latestImageMutex);
	  if (latestStampedImage == nullptr) {
		return StampedImage(previewImageWithText("Waiting for the first image to arrive"));
	  }
	  stampedImage = latestStampedImage->clone();
	  if (stampedImage.image.channels() == 1) {
		cv::cvtColor(stampedImage.image, stampedImage.image, cv::COLOR_GRAY2BGR);
	  }
	}

    // Get the latest observation
	boost::shared_ptr<aslam::cameras::GridCalibrationTargetObservation> observation = nullptr;
	{
	  std::lock_guard<std::mutex> lock(targetObservationsMutex);
	  ss << "Collected " << targetObservations->size() << " images";
	  auto it = targetObservations->find(stampedImage.timestamp);
	  if (it != targetObservations->end()) {
		observation = boost::make_shared<aslam::cameras::GridCalibrationTargetObservation>(it->second);
	  }
	}

	cv::putText(stampedImage.image,
				ss.str(),
				cv::Point(20, 40),
				cv::FONT_HERSHEY_DUPLEX,
				1.0,
				cv::Scalar(255, 0, 0),
				2);

	if (observation != nullptr && observation->time().toDvTime() == stampedImage.timestamp) {
	  cv::Point prevPoint(-1, -1);
	  for (size_t y = 0; y < grid->rows(); ++y) {
		const auto color = colors[y % colors.size()];
		for (size_t x = 0; x < grid->cols(); ++x) {
		  const auto idx = y * grid->cols() + x;
		  Eigen::Vector2d point;
		  if (observation->imagePoint(idx, point)) {
			const cv::Point cvPoint(point.x(), point.y());
			cv::circle(stampedImage.image, cvPoint, 8, color, 2);
			if (prevPoint != cv::Point(-1, -1)) {
			  cv::line(stampedImage.image, prevPoint, cvPoint, color, 2);
			}
			prevPoint = cvPoint;
		  }
		}
	  }
	}

	return stampedImage;
  }
  case CALIBRATING:
    // TODO(radam):
    break;
  case CALIBRATED:
    // TODO(radam):
    break;
  default:
    throw std::runtime_error("Unknown state");
  }

  // TODO(radam): del
  return StampedImage(previewImageWithText("This should never happen"));
}

size_t Calibrator::getNumDetections() {
  std::lock_guard<std::mutex> lock(targetObservationsMutex);
  return targetObservations->size();
}

void Calibrator::calibrate()  {
  state = CALIBRATING;

  // TODO(radam): everything below should happen in a separate thread

  std::cout << "Waiting for the detector to finish..." << std::endl;
  detectionsQueue.waitForEmptyQueue();
  detectionsQueue.join();

  std::lock_guard<std::mutex> lock(targetObservationsMutex);
  std::cout << "Calibrating using " << targetObservations->size() << " detections." << std::endl;

  const size_t maxIter = 300; // TODO(radam): param
  iccCalibrator->buildProblem(6,
							 70,
							 70,
							 false,
							 1e6,
							 1e5,
							 true,
							 -1,
							 -1,
							 -1,
							 true,
							 true,
							 maxIter,
							 1.0,
							 1.0,
							 0.02,
							 false);
  iccCalibrator->optimize(nullptr, maxIter, false);
}


void Calibrator::detectPattern(const StampedImage &stampedImage) {

// TODO(radam): read input image and undistort points before passing to kalibr

  std::vector<cv::Point2f> pointBuf;

  // Make a copy of the detector in each thread to avoid memory issues
  auto detector = boost::make_shared<aslam::cameras::GridDetector>(iccCamera->getCameraGeometry(), grid, detectorOptions);

  // Search for pattern and draw it on the image frame
  aslam::cameras::GridCalibrationTargetObservation observation;
  bool success = detector->findTarget(stampedImage.image, aslam::Time(toSec(stampedImage.timestamp)), observation);

  // If pattern detected add it to observation
  if (success) {
	if (observation.hasSuccessfulObservation()) {
		std::lock_guard<std::mutex> lock(targetObservationsMutex);
		targetObservations->emplace(stampedImage.timestamp, observation);
	}
  }

  // Replace the most recent image even if no pattern detected
  std::lock_guard<std::mutex> lock(latestImageMutex);
  bool replace = true;
  if (latestStampedImage != nullptr) {
	if (stampedImage.timestamp < latestStampedImage->timestamp) {
	  replace = false;
	}
  }
  if (replace) {
	latestStampedImage = boost::make_shared<StampedImage>(stampedImage.clone());
  }
}
