//
// Created by radam on 2021-03-23.
//

#pragma once

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

protected:

  // Calibration settings
  cv::Size boardSize;
  enum CalibrationPattern {
    CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
  };
  CalibrationPattern calibrationPattern;

  // Queue scheduling pattern detection jobs
  sm::JobQueue detectionsQueue;

  // Preview image
  std::mutex previewMutex;
  cv::Mat previewImage;
  int64_t previewTimestamp = 0LL;


public:
  /**
   * Constructor.
   */
  Calibrator();

  /**
   * Add IMU measurement to the calibration buffer.
   */
  void addImu();


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