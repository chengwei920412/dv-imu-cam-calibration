//
// Created by radam on 2021-03-23.
//

#pragma once

#include <kalibr_imu_camera_calibration/iccCalibrator.hpp>
#include <kalibr_imu_camera_calibration/iccSensors.hpp>

#include <aslam/cameras/GridDetector.hpp>
#include <sm/boost/JobQueue.hpp>

#include <opencv2/opencv.hpp>

#include <atomic>
#include <mutex>

/**
 * IMU camera calibration.
 */
class Calibrator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

    enum CalibrationPattern { CHESSBOARD, ASYMMETRIC_CIRCLES_GRID, APRIL_GRID };

    enum State { INITIALIZED, COLLECTING, COLLECTED, CALIBRATING, CALIBRATED };

    struct Options {
        // Calibration pattern
        size_t rows = 11;
        size_t cols = 4;
        double spacingMeters = 0.05;
        double tagSpacing = 0.3;
        CalibrationPattern pattern = CalibrationPattern::ASYMMETRIC_CIRCLES_GRID;

        // Optimization problem
        size_t maxIter = 20;
        bool timeCalibration = true;

        // IMU
        ImuParameters imuParameters;

        // Camera
        std::vector<double> intrinsics{
            6.3007006020163419e+02,
            6.3066578517514370e+02,
            2.9440730698720517e+02,
            2.5090606048734924e+02};
        std::vector<double> distCoeffs{
            -3.9458674051766940e-01,
            4.2159874612649451e-01,
            2.3858518861790620e-03,
            -2.9775941904103447e-03};
        cv::Size imageSize{640, 480};
    };

protected:
    // Internal state
    std::atomic<State> state;

    // Calibration options
    const Options calibratorOptions;

    // Queue scheduling pattern detection jobs
    sm::JobQueue detectionsQueue;

    // IMU data
    boost::shared_ptr<std::vector<ImuMeasurement>> imuData = nullptr;
    std::mutex imuDataMutex;

    // IccCalibrator
    boost::shared_ptr<IccCalibrator> iccCalibrator = nullptr;
    boost::shared_ptr<IccImu> iccImu = nullptr;
    boost::shared_ptr<IccCamera> iccCamera = nullptr;

    // Calibration target grid detector
    aslam::cameras::GridDetector::GridDetectorOptions detectorOptions;
    boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> grid = nullptr;
    boost::shared_ptr<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>> targetObservations = nullptr;
    std::mutex targetObservationsMutex;

    // Latest image used for visualization
    boost::shared_ptr<StampedImage> latestStampedImage = nullptr;
    boost::shared_ptr<aslam::cameras::GridCalibrationTargetObservation> latestObservation = nullptr;
    std::mutex latestImageMutex;

    // Colors used in visualization of detected calibtion pattern
    const std::vector<cv::Scalar> colors{
        cv::Scalar(255, 0, 0),   cv::Scalar(128, 0, 0),   cv::Scalar(0, 255, 0),   cv::Scalar(0, 128, 0),
        cv::Scalar(0, 0, 255),   cv::Scalar(0, 0, 128),   cv::Scalar(255, 255, 0), cv::Scalar(128, 255, 0),
        cv::Scalar(0, 255, 255), cv::Scalar(0, 255, 128), cv::Scalar(255, 128, 0), cv::Scalar(128, 128, 0),
        cv::Scalar(0, 128, 255), cv::Scalar(0, 128, 128), cv::Scalar(255, 0, 255), cv::Scalar(128, 0, 255),
        cv::Scalar(0, 255, 255), cv::Scalar(0, 128, 255), cv::Scalar(255, 0, 128), cv::Scalar(128, 0, 128),
        cv::Scalar(0, 255, 128), cv::Scalar(0, 128, 128)};

public:
    /**
     * Constructor.
     */
    Calibrator(const Options& opts);

    /**
     * Destructor
     */
    ~Calibrator();

    /**
     * Add IMU measurement to the calibration buffer.
     */
    void addImu(
        int64_t timestamp,
        double gyroX,
        double gyroY,
        double gyroZ,
        double accelX,
        double accelY,
        double accelZ);

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

    /**
     * Begin calibration procedure on the collected data.
     *
     * @return result of the calibration
     */
    [[nodiscard]] IccCalibrator::CalibrationResult calibrate();

    /**
     * Start collecting data from the camera and IMU.
     */
    void startCollecting();

    /**
     * Stop collecting data from the camera and IMU.
     */
    void stopCollecting();

    /**
     * Discard the collected data and reset the calibrator.
     */
    void reset();

protected:
    /**
     * Detect the calibration pattern on the given stamped image.
     *
     * @param stampedImage
     */
    void detectPattern(const StampedImage& stampedImage);
};
