#pragma once

#include "kalibr_calibrate_cameras/CameraCalibrator.hpp"
#include "kalibr_imu_camera_calibration/iccCalibrator.hpp"
#include "kalibr_imu_camera_calibration/iccCamera.hpp"
#include "kalibr_imu_camera_calibration/iccImu.hpp"

#include <aslam/cameras.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <sm/boost/JobQueue.hpp>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>
#include <atomic>
#include <dv-processing/exception/exception.hpp>
#include <dv-processing/kinematics/transformation.hpp>
#include <iostream>
#include <mutex>
#include <string>
#include <tbb/parallel_for_each.h>

namespace CalibratorUtils {
double toSec(const int64_t time) {
    return static_cast<double>(time) / 1e6;
}
/**
 * Hold image and corresponding timestamp.
 */
struct StampedImage {
    cv::Mat image;
    int64_t timestamp;

    StampedImage(){};

    StampedImage(cv::Mat img, const int64_t ts) : image(std::move(img)), timestamp(ts){};

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

enum PatternType { CHESSBOARD, ASYMMETRIC_CIRCLES_GRID, APRIL_GRID };

enum State { INITIALIZED, COLLECTING, COLLECTED, CALIBRATING, CALIBRATED };

struct Options {
    // Calibration pattern
    size_t rows = 11;
    size_t cols = 4;
    double spacingMeters = 0.05;
    double patternSpacing = 0.3;
    PatternType pattern = PatternType::ASYMMETRIC_CIRCLES_GRID;

    // Optimization problem
    size_t maxIter = 20;
    bool timeCalibration = true;

    // IMU
    ImuParameters imuParameters;

    // Camera
    struct CameraInits {
        std::vector<double> intrinsics;
        std::vector<double> distCoeffs;
        cv::Size imageSize;
    };

    std::vector<CameraInits> cameraInitialSettings;
};

StampedImage previewImageWithText(
    const std::string& text,
    const int64_t timestamp = 0LL,
    const cv::Size& size = cv::Size(640, 480)) {
    cv::Mat img = cv::Mat::zeros(size, CV_8UC3);

    cv::putText(
        img,
        text,
        cv::Point(size.width / 8, size.height / 2),
        cv::FONT_HERSHEY_DUPLEX,
        1.0,
        cv::Scalar(255, 255, 255),
        2);

    return {img, timestamp};
}

} // namespace CalibratorUtils

/**
 * IMU camera calibration.
 */
class CalibratorBase {
public:
    virtual ~CalibratorBase() = default;
    /**
     * Add IMU measurement to the calibration buffer.
     */
    virtual void addImu(const int64_t timestamp, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc) = 0;

    /**
     * Add a stamped image to the calibration buffer.
     *
     * @param CalibratorUtils::StampedImage
     */
    virtual void addImages(const std::vector<CalibratorUtils::StampedImage>& stampedImages) = 0;

    /**
     * Return the last images and grid observations
     * @return a pair containing a vector of CalibratorUtils::StampedImage and a vector of pointers to
     * aslam::cameras::GridCalibrationTargetObservation
     */
    virtual std::pair<
        std::vector<CalibratorUtils::StampedImage>,
        std::vector<boost::shared_ptr<aslam::cameras::GridCalibrationTargetObservation>>>
        getLatestObservations() = 0;
    
    /**
     * @return preview image visualizing the current status of the calibration
     */
    virtual std::vector<CalibratorUtils::StampedImage> getPreviewImages() = 0;

    /**
     * Calibrate the camera intrinsics (monocular).
     */
    virtual std::optional<std::vector<CameraCalibrationUtils::CalibrationResult>> calibrateCameraIntrinsics() = 0;

    /**
     * Build optimization problem. Needs to be called before calibrate().
     */
    virtual void buildProblem() = 0;

    /**
     * Begin calibration procedure on the collected data.
     *
     * @return result of the calibration
     */
    [[nodiscard]] virtual IccCalibratorUtils::CalibrationResult calibrate() = 0;

    /**
     * Start collecting data from the camera and IMU.
     */
    virtual void startCollecting() = 0;

    /**
     * Stop collecting data from the camera and IMU.
     */
    virtual void stopCollecting() = 0;

    /**
     * Discard the collected data and reset the calibrator.
     */
    virtual void reset() = 0;

    /**
     * Get a string containing DV log information before optimization.
     *
     * @param ss string stream into which log will be output
     */
    virtual void getDvInfoBeforeOptimization(std::ostream& ss) = 0;

    /**
     * Get a string containing DV log information after optimization.
     *
     * @param ss string stream into which log will be output
     */
    virtual void getDvInfoAfterOptimization(std::ostream& ss) = 0;

    virtual std::ostream& print(std::ostream& os) = 0;

protected:
    /**
     * Detect the calibration pattern on the given stamped image.
     *
     * @param stampedImage
     */
    virtual void detectPattern(const std::vector<CalibratorUtils::StampedImage>& frames) = 0;
};