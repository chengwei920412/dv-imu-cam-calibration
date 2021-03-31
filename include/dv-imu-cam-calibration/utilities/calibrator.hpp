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
  std::vector<ImuMeasurement> imuData;
  ImuParameters imuParameters; // TODO(radam): take them as input

  // IccCalibrator
  IccCalibrator iccCalibrator;
  IccImu iccImu;
  IccCamera iccCamera;

  // Calibration target grid detector
  aslam::cameras::GridDetector::GridDetectorOptions detectorOptions;
  boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> grid = nullptr;

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
