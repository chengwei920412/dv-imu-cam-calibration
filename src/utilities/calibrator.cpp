//
// Created by radam on 2021-03-23.
//

#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>

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

Calibrator::Calibrator(const Options& opts) : calibratorOptions(opts) {

  targetObservations = boost::make_shared<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>>();
  imuData = boost::make_shared<std::vector<ImuMeasurement>>();

  iccImu = boost::make_shared<IccImu>(calibratorOptions.imuParameters, imuData);
  iccCamera = boost::make_shared<IccCamera>(targetObservations);
  iccCalibrator = boost::make_shared<IccCalibrator>(iccCamera, iccImu);

  detectionsQueue.start(std::max(1u, std::thread::hardware_concurrency()-1));

  std::cout << "Initializing calibration target:" << std::endl;

  switch (calibratorOptions.pattern) {
  case CalibrationPattern::CHESSBOARD: {
    std::cout << "  Type: CHESSBOARD" << std::endl;
    std::cout << "  Rows:" << std::endl;
    std::cout << "    Count: " << calibratorOptions.rows << std::endl;
    std::cout << "    Distance: " << calibratorOptions.spacingMeters << " [m]" << std::endl;
	std::cout << "  Cols:" << std::endl;
	std::cout << "    Count: " << calibratorOptions.cols << std::endl;
	std::cout << "    Distance: " << calibratorOptions.spacingMeters << " [m]" << std::endl;

	aslam::cameras::GridCalibrationTargetCheckerboard::CheckerboardOptions options;
	options.filterQuads = true;
	options.normalizeImage = true;
	options.useAdaptiveThreshold = true;
	options.performFastCheck = false;
	options.windowWidth = 5;
	options.showExtractionVideo = false;

	grid = boost::make_shared<aslam::cameras::GridCalibrationTargetCheckerboard>(calibratorOptions.rows,
																				 calibratorOptions.cols,
																				 calibratorOptions.spacingMeters,
																				 calibratorOptions.spacingMeters,
																				 options);
	break;
  }
  case CalibrationPattern::ASYMMETRIC_CIRCLES_GRID: {
	std::cout << "  Type: ASYMMETRIC_CIRCLES_GRID" << std::endl;
	std::cout << "  Rows:" << std::endl;
	std::cout << "    Count: " << calibratorOptions.rows << std::endl;
	std::cout << "  Cols:" << std::endl;
	std::cout << "    Count: " << calibratorOptions.cols << std::endl;
	std::cout << "  Spacing: " << calibratorOptions.spacingMeters << " [m]" <<  std::endl;

	aslam::cameras::GridCalibrationTargetCirclegrid::CirclegridOptions options;
	options.showExtractionVideo = false;
	options.useAsymmetricCirclegrid = true;

	grid = boost::make_shared<aslam::cameras::GridCalibrationTargetCirclegrid>(calibratorOptions.rows,
																			   calibratorOptions.cols,
																			   calibratorOptions.spacingMeters,
																			   options);
    break;
  }
  case CalibrationPattern::APRIL_GRID: {
	std::cout << "  Type: APRIL_GRID" << std::endl;
	std::cout << "  Tags:" << std::endl;
	std::cout << "    Cols: " << calibratorOptions.cols << std::endl;
	std::cout << "    Rows: " << calibratorOptions.rows << std::endl;
	std::cout << "    Size: " << calibratorOptions.spacingMeters << " [m]" << std::endl;
	std::cout << "    Spacing : "  << calibratorOptions.spacingMeters * calibratorOptions.tagSpacing << " [m]" << std::endl;

	aslam::cameras::GridCalibrationTargetAprilgrid::AprilgridOptions options;
    // enforce more than one row --> pnp solution can be bad if all points are almost on a line...
	options.minTagsForValidObs = std::max(calibratorOptions.rows, calibratorOptions.cols) + 1;
	options.showExtractionVideo = false;

	grid = boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(calibratorOptions.rows,
																			  calibratorOptions.cols,
																			  calibratorOptions.spacingMeters,
																			  calibratorOptions.tagSpacing,
																			  options);
    break;
  }
  default:
    throw std::runtime_error("Not implemented calibration pattern");
  }

  detectorOptions.imageStepping = false;
  detectorOptions.plotCornerReprojection = false;
  detectorOptions.filterCornerOutliers = true;

  state = INITIALIZED;
}

Calibrator::~Calibrator() {
  detectionsQueue.join();
}

void Calibrator::addImu(const int64_t timestamp,
						const double gyroX,
						const double gyroY,
						const double gyroZ,
						const double accelX,
						const double accelY,
						const double accelZ) {
  const double tsS = static_cast<double>(timestamp) / 1e6;
  const auto Rgyro = Eigen::Matrix3d::Identity() * iccImu->getGyroUncertaintyDiscrete();
  const auto Raccel = Eigen::Matrix3d::Identity() * iccImu->getAccelUncertaintyDiscrete();
  const Eigen::Vector3d omega(gyroX, gyroY, gyroZ);
  const Eigen::Vector3d alpha(accelX, accelY, accelZ);
  ImuMeasurement imuMeas(tsS, omega, alpha, Rgyro, Raccel);

  if (state == COLLECTING) {
	std::lock_guard<std::mutex> lock(imuDataMutex);
	imuData->push_back(imuMeas);
  }

}




void Calibrator::addImage(const StampedImage& stampedImage) {
  assert(stampedImage.image.channels() == 1 && "calibrator expect a grayscale image");

  boost::unique_future<void> job;
  detectionsQueue.scheduleFuture<void>(boost::bind(&Calibrator::detectPattern, this, stampedImage.clone()), job);
}

void Calibrator::addImage(const cv::Mat &img, int64_t timestamp) {
  addImage(StampedImage(img, timestamp));
}

Calibrator::StampedImage Calibrator::getPreviewImage() {

  // Get the latest image
  StampedImage stampedImage;
  {
	std::lock_guard<std::mutex> lock(latestImageMutex);
	if (latestStampedImage == nullptr) {
	  return StampedImage(previewImageWithText("No image arrived yet"));
	}
	stampedImage = latestStampedImage->clone();
	if (stampedImage.image.channels() == 1) {
	  cv::cvtColor(stampedImage.image, stampedImage.image, cv::COLOR_GRAY2BGR);
	}
  }

  switch (state) {
  case INITIALIZED: {
	cv::putText(stampedImage.image,
				"Ready to collect",
				cv::Point(20, 40),
				cv::FONT_HERSHEY_DUPLEX,
				1.0,
				cv::Scalar(255, 0, 0),
				2);
	break;
  }
  case COLLECTING: {
	// Get the latest observation
	std::stringstream ss;
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
	break;
  }
  case CALIBRATING: {
	cv::putText(stampedImage.image,
				"Calibrating...",
				cv::Point(20, 40),
				cv::FONT_HERSHEY_DUPLEX,
				1.0,
				cv::Scalar(255, 0, 0),
				2);
	break;
  }
  case CALIBRATED:
	cv::putText(stampedImage.image,
				"Calibrated!",
				cv::Point(20, 40),
				cv::FONT_HERSHEY_DUPLEX,
				1.0,
				cv::Scalar(255, 0, 0),
				2);
    break;
  default:
    throw std::runtime_error("Unknown state");
  }

  return stampedImage;
}

size_t Calibrator::getNumDetections() {
  std::lock_guard<std::mutex> lock(targetObservationsMutex);
  return targetObservations->size();
}

void Calibrator::calibrate()  {


  // TODO(radam): everything below should happen in a separate thread

  std::cout << "Waiting for the detector to finish..." << std::endl;
  detectionsQueue.waitForEmptyQueue();

  state = CALIBRATING;

  // Block all the threads
  std::lock_guard<std::mutex> lock1(targetObservationsMutex);
  std::lock_guard<std::mutex> lock2(imuDataMutex);

  if (targetObservations->empty()) {
    std::cout << "No observations collected" << std::endl;
    return;
  }
  
  std::cout << "Calibrating using " << targetObservations->size() << " detections." << std::endl;


  std::cout << std::endl << "Building the problem" << std::endl;
  iccCalibrator->buildProblem(6,
							 100,
							 50,
							 false,
							 1e6,
							 1e5,
							 true,
							 -1,
							 -1,
							 -1,
							 !calibratorOptions.timeCalibration,
							 true,
							  calibratorOptions.maxIter,
							 1.0,
							 1.0,
							 0.03,
							 false);

  //  // Printing reprojection errors is quite difficult, not doing it
  //  std::cout << std::endl << "Before Optimization" << std::endl << "###################" << std::endl;
  //  iccCalibrator->printErrorStatistics();

  std::cout << std::endl << "Optimizing..." << std::endl;
  iccCalibrator->optimize(nullptr, calibratorOptions.maxIter, false);

  //  // Printing reprojection errors is quite difficult, not doing it
  //  std::cout << std::endl << "After Optimization" << std::endl << "###################" << std::endl;
  //  iccCalibrator->printErrorStatistics();
  
  std::cout << std::endl << "Results" << std::endl << "#######" << std::endl;
  iccCalibrator->printResult();

  state = CALIBRATED;
}


void Calibrator::detectPattern(const StampedImage &stampedImage) {

  std::vector<cv::Point2f> pointBuf;

  // Make a copy of the detector in each thread to avoid memory issues
  auto detector = boost::make_shared<aslam::cameras::GridDetector>(iccCamera->getCameraGeometry(), grid, detectorOptions);

  // Search for pattern and draw it on the image frame
  aslam::cameras::GridCalibrationTargetObservation observation;
  bool success = detector->findTarget(stampedImage.image, aslam::Time(toSec(stampedImage.timestamp)), observation);


  // If pattern detected add it to observation
  if (state == COLLECTING && success && observation.hasSuccessfulObservation()) {
	std::lock_guard<std::mutex> lock(targetObservationsMutex);
	targetObservations->emplace(stampedImage.timestamp, observation);
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

void Calibrator::startCollecting() {
  detectionsQueue.waitForEmptyQueue();

  {
	std::lock_guard<std::mutex> lock1(targetObservationsMutex);
	targetObservations->clear();
  }

  {
	std::lock_guard<std::mutex> lock2(imuDataMutex);
	imuData->clear();
  }

  state = COLLECTING;
}
