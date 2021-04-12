//
// Created by radam on 2021-03-23.
//

#pragma once

#include <kalibr_imu_camera_calibration/iccCalibrator.hpp>
#include <kalibr_imu_camera_calibration/iccSensors.hpp>

#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <aslam/cameras/GridDetector.hpp>

#include <opencv2/opencv.hpp>

#include <sm/boost/JobQueue.hpp>

#include <mutex>

/**
 * IMU camera calibration.
 */
class Calibrator {

public:

  /**
   * Hold image and corresponding timestamp.
   */
  struct StampedImage {
	cv::Mat image;
	int64_t timestamp;

	StampedImage() {};

	StampedImage(cv::Mat img, const int64_t ts) : image(std::move(img)), timestamp(ts) {};

	/**
	 * Clone the underlying image.
	 *
	 * @return
	 */
	StampedImage clone() const {
	  StampedImage clone;
	  clone.image = image.clone();
	  clone.timestamp = timestamp;
	  return clone;
	}
  };

  enum CalibrationPattern {
	CHESSBOARD, ASYMMETRIC_CIRCLES_GRID, APRIL_GRID
  };

protected:

  // Calibration settings
  cv::Size boardSize;
  CalibrationPattern calibrationPattern;

  // Queue scheduling pattern detection jobs
  sm::JobQueue detectionsQueue;

  // Preview image
  std::mutex previewMutex;
  cv::Mat previewImage;
  int64_t previewTimestamp = 0LL;

  // IMU data
  boost::shared_ptr<std::vector<ImuMeasurement>> imuData = nullptr;
  ImuParameters imuParameters; // TODO(radam): take them as input

  // IccCalibrator
  IccCalibrator iccCalibrator;
  boost::shared_ptr<IccImu> iccImu = nullptr;
  boost::shared_ptr<IccCamera> iccCamera = nullptr;

  // Calibration target grid detector
  aslam::cameras::GridDetector::GridDetectorOptions detectorOptions;
  boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> grid = nullptr;
  boost::shared_ptr<std::vector<aslam::cameras::GridCalibrationTargetObservation>> targetObservations = nullptr;
  std::mutex targetObservationsMutex;

  // Colors used in visualization of detected calibtion pattern
  const std::vector<cv::Scalar> colors{
	  cv::Scalar(255, 0, 0),
	  cv::Scalar(128, 0, 0),
	  cv::Scalar(0, 255, 0),
	  cv::Scalar(0, 128, 0),
	  cv::Scalar(0, 0, 255),
	  cv::Scalar(0, 0, 128),
	  cv::Scalar(255, 255, 0),
	  cv::Scalar(128, 255, 0),
	  cv::Scalar(0, 255, 255),
	  cv::Scalar(0, 255, 128),
	  cv::Scalar(255, 128, 0),
	  cv::Scalar(128, 128, 0),
	  cv::Scalar(0, 128, 255),
	  cv::Scalar(0, 128, 128),
	  cv::Scalar(255, 0, 255),
	  cv::Scalar(128, 0, 255),
	  cv::Scalar(0, 255, 255),
	  cv::Scalar(0, 128, 255),
	  cv::Scalar(255, 0, 128),
	  cv::Scalar(128, 0, 128),
	  cv::Scalar(0, 255, 128),
	  cv::Scalar(0, 128, 128)
  };

public:
  /**
   * Constructor.
   */
  Calibrator();

  /**
   * Add IMU measurement to the calibration buffer.
   */
  void addImu(int64_t timestamp,
			  double gyroX,
			  double gyroY,
			  double gyroZ,
			  double accelX,
			  double accelY,
			  double accelZ
			  );


  /**
   * Add a stamped image to the calibration buffer.
   *
   * @param stampedImage
   */
  void addImage(const StampedImage& stampedImage);

  void addImage(const cv::Mat& img, int64_t timestamp);

  /**
   * @return preview image visualizing the current status of the calibration
   */
  StampedImage getPreviewImage();

  // TODO(radam): think how to nicely handle mutexes and stuff
  // TODO(radam): rethink this
  void calibrate() {
	const size_t maxIter = 30;
	iccCalibrator.buildProblem(4,
							   100,
							   50,
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
							   30.e-3,
							   false);
	iccCalibrator.optimize(nullptr, maxIter, false);
  }

protected:

  /**
   * Set the preview image to the given stamped image.
   *
   * @param stampedImage
   */
  void setPreviewImage(const StampedImage& stampedImage);

  /**
   * Detect the calibration pattern on the given stamped image.
   *
   * @param stampedImage
   */
  void detectPattern(const StampedImage& stampedImage);



};
