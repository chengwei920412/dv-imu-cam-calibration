//
// Created by radam on 2021-03-23.
//

#pragma once

#include <kalibr_imu_camera_calibration/iccCalibrator.hpp>
#include <kalibr_imu_camera_calibration/iccSensors.hpp>

#include <aslam/cameras.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <kalibr_calibrate_cameras/CameraCalibrator.hpp>
#include <sm/boost/JobQueue.hpp>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>
#include <atomic>
#include <dv-processing/exception/exception.hpp>
#include <dv-processing/kinematics/transformation.hpp>
#include <mutex>
#include <tbb/parallel_for_each.h>

namespace CalibratorUtils {

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

    return StampedImage(img, timestamp);
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

protected:
    /**
     * Detect the calibration pattern on the given stamped image.
     *
     * @param stampedImage
     */
    virtual void detectPattern(const std::vector<CalibratorUtils::StampedImage>& frames) = 0;
};

/**
 * IMU camera calibration.
 */
template<typename CameraGeometryType, typename DistortionType>
class Calibrator : public CalibratorBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    // Internal state
    std::atomic<CalibratorUtils::State> state;

    // Calibration options
    CalibratorUtils::Options calibratorOptions;

    // Queue scheduling pattern detection jobs
    sm::JobQueue detectionsQueue;

    // IMU data
    boost::shared_ptr<std::vector<IccSensors::ImuMeasurement>> imuData = nullptr;
    std::mutex imuDataMutex;

    // IccCalibrator
    boost::shared_ptr<IccCalibrator<CameraGeometryType, DistortionType>> iccCalibrator = nullptr;
    boost::shared_ptr<IccImu> iccImu = nullptr;
    std::vector<boost::shared_ptr<IccCamera<CameraGeometryType, DistortionType>>> iccCameras;

    // Calibration target grid detector
    aslam::cameras::GridDetector::GridDetectorOptions detectorOptions;
    boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> grid = nullptr;
    std::map<size_t, IccCameraUtils::ObservationsPtr> camTargetObservations;
    std::mutex targetObservationsMutex;

    // Latest image used for visualization
    std::vector<CalibratorUtils::StampedImage> latestImages;
    std::vector<boost::shared_ptr<aslam::cameras::GridCalibrationTargetObservation>> latestObservations;
    std::mutex latestImageMutex;

    std::vector<boost::shared_ptr<aslam::cameras::GridDetector>> detectors;

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
    Calibrator(const CalibratorUtils::Options& opts) : calibratorOptions(opts) {
        imuData = boost::make_shared<std::vector<IccSensors::ImuMeasurement>>();
        std::cout << "Initializing calibration target:" << std::endl;

        switch (calibratorOptions.pattern) {
            case CalibratorUtils::PatternType::CHESSBOARD: {
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

                grid = boost::make_shared<aslam::cameras::GridCalibrationTargetCheckerboard>(
                    calibratorOptions.rows,
                    calibratorOptions.cols,
                    calibratorOptions.spacingMeters,
                    calibratorOptions.spacingMeters,
                    options);
                break;
            }
            case CalibratorUtils::PatternType::ASYMMETRIC_CIRCLES_GRID: {
                std::cout << "  Type: ASYMMETRIC_CIRCLES_GRID" << std::endl;
                std::cout << "  Rows:" << std::endl;
                std::cout << "    Count: " << calibratorOptions.rows << std::endl;
                std::cout << "  Cols:" << std::endl;
                std::cout << "    Count: " << calibratorOptions.cols << std::endl;
                std::cout << "  Spacing: " << calibratorOptions.spacingMeters << " [m]" << std::endl;

                aslam::cameras::GridCalibrationTargetCirclegrid::CirclegridOptions options;
                options.showExtractionVideo = false;
                options.useAsymmetricCirclegrid = true;

                grid = boost::make_shared<aslam::cameras::GridCalibrationTargetCirclegrid>(
                    calibratorOptions.rows,
                    calibratorOptions.cols,
                    calibratorOptions.spacingMeters,
                    options);
                break;
            }
            case CalibratorUtils::PatternType::APRIL_GRID: {
                std::cout << "  Type: APRIL_GRID" << std::endl;
                std::cout << "  Tags:" << std::endl;
                std::cout << "    Cols: " << calibratorOptions.cols << std::endl;
                std::cout << "    Rows: " << calibratorOptions.rows << std::endl;
                std::cout << "    Size: " << calibratorOptions.spacingMeters << " [m]" << std::endl;
                std::cout << "    Spacing : " << calibratorOptions.spacingMeters * calibratorOptions.patternSpacing
                          << " [m]" << std::endl;

                aslam::cameras::GridCalibrationTargetAprilgrid::AprilgridOptions options;
                // enforce more than one row --> pnp solution can be bad if all points are almost on a line...
                options.minTagsForValidObs = std::max(calibratorOptions.rows, calibratorOptions.cols) + 1;
                options.showExtractionVideo = false;

                grid = boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(
                    calibratorOptions.rows,
                    calibratorOptions.cols,
                    calibratorOptions.spacingMeters,
                    calibratorOptions.patternSpacing,
                    options);
                break;
            }
            default: throw std::runtime_error("Not implemented calibration pattern");
        }

        detectorOptions.imageStepping = false;
        detectorOptions.plotCornerReprojection = false;
        detectorOptions.filterCornerOutliers = true;

        size_t camId = 0;
        for (auto& cameraOptions : calibratorOptions.cameraInitialSettings) {
            if (cameraOptions.imageSize.area() == 0) {
                throw std::runtime_error("Image size given to the calibrator is incorrect. The area is equal zero.");
            }

            // Guess the intrinsics of the camera
            if (cameraOptions.intrinsics.empty()) {
                cameraOptions.intrinsics.push_back(cameraOptions.imageSize.width);
                cameraOptions.intrinsics.push_back(cameraOptions.imageSize.width);
                cameraOptions.intrinsics.push_back(cameraOptions.imageSize.width / 2);
                cameraOptions.intrinsics.push_back(cameraOptions.imageSize.height / 2);
            }

            // Guess the distortion coefficients
            if (cameraOptions.distCoeffs.empty()) {
                cameraOptions.distCoeffs.push_back(0.0);
                cameraOptions.distCoeffs.push_back(0.0);
                cameraOptions.distCoeffs.push_back(0.0);
                cameraOptions.distCoeffs.push_back(0.0);
            }

            auto targetObservations
                = boost::make_shared<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>>();
            const auto iccCamera = boost::make_shared<IccCamera<CameraGeometryType, DistortionType>>(
                cameraOptions.intrinsics,
                cameraOptions.distCoeffs,
                cameraOptions.imageSize,
                targetObservations);
            iccCameras.push_back(iccCamera);

            camTargetObservations.insert(std::make_pair(camId, targetObservations));

            detectors.push_back(boost::make_shared<aslam::cameras::GridDetector>(
                iccCamera->getCameraGeometry(),
                grid,
                detectorOptions));
            camId++;
        }

        assert(iccCameras.size()>0);

        iccImu = boost::make_shared<IccImu>(calibratorOptions.imuParameters, imuData);

        // Use C0 camera for imu calibration
        iccCalibrator
            = boost::make_shared<IccCalibrator<CameraGeometryType, DistortionType>>(iccCameras.front(), iccImu);

        detectionsQueue.start(std::max(1u, std::thread::hardware_concurrency() - 1));

        state = CalibratorUtils::INITIALIZED;
    }

    /**
     * Destructor
     */
    ~Calibrator() override {
        detectionsQueue.join();
    }

    /**
     * Add IMU measurement to the calibration buffer.
     */
    void addImu(const int64_t timestamp, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc) override {
        const double tsS = static_cast<double>(timestamp) / 1e6;
        const auto Rgyro = Eigen::Matrix3d::Identity() * iccImu->getGyroUncertaintyDiscrete();
        const auto Raccel = Eigen::Matrix3d::Identity() * iccImu->getAccelUncertaintyDiscrete();
        IccSensors::ImuMeasurement imuMeas(tsS, gyro, acc, Rgyro, Raccel);

        if (state == CalibratorUtils::COLLECTING) {
            std::lock_guard<std::mutex> lock(imuDataMutex);
            imuData->push_back(imuMeas);
        }
    }

    /**
     * Add a stamped image to the calibration buffer.
     *
     * @param stampedImage
     */
    void addImages(const std::vector<CalibratorUtils::StampedImage>& stampedImages) override {
        for (const auto& stampedImage : stampedImages) {
            if (stampedImage.image.channels() != 1) {
                throw dv::exceptions::RuntimeError("Calibrator expects grayscale images!");
            }
        }

        if (state == CalibratorUtils::COLLECTING) {
            boost::unique_future<void> job;
            std::vector<CalibratorUtils::StampedImage> images;
            for (const auto& img : stampedImages) {
                images.push_back(img.clone());
            }
            detectionsQueue.scheduleFuture<void>(
                [this, images] {
                    detectPattern(images);
                },
                job);
        } else {
            std::lock_guard<std::mutex> lock(latestImageMutex);
            latestImages = stampedImages;
            latestObservations.clear();
        }
    }

    std::pair<
        std::vector<CalibratorUtils::StampedImage>,
        std::vector<boost::shared_ptr<aslam::cameras::GridCalibrationTargetObservation>>>
        getLatestObservations() override {
        return std::make_pair(latestImages, latestObservations);
    }
    /**
     * @return preview image visualizing the current status of the calibration
     */
    std::vector<CalibratorUtils::StampedImage> getPreviewImages() override {
        // Get the latest image
        std::vector<CalibratorUtils::StampedImage> previews;
        std::vector<boost::shared_ptr<aslam::cameras::GridCalibrationTargetObservation>> latestObs;
        {
            std::lock_guard<std::mutex> lock(latestImageMutex);
            if (latestImages.empty()) {
                for (const auto& camera : calibratorOptions.cameraInitialSettings) {
                    previews.push_back(CalibratorUtils::StampedImage(
                        CalibratorUtils::previewImageWithText("No image arrived yet", 0LL, camera.imageSize)));
                }
                return previews;
            }
            for (const auto& image : latestImages) {
                if (image.image.channels() == 1) {
                    cv::Mat colored;
                    cv::cvtColor(image.image, colored, cv::COLOR_GRAY2BGR);
                    previews.emplace_back(colored, image.timestamp);
                } else {
                    previews.push_back(image.clone());
                }
            }
            latestObs = latestObservations;
        }

        size_t cameraId = 0;
        for (auto& latestImage : previews) {
            boost::shared_ptr<aslam::cameras::GridCalibrationTargetObservation> obs
                = latestObs.empty() ? nullptr : latestObs[cameraId];
            switch (state) {
                case CalibratorUtils::INITIALIZED: {
                    cv::putText(
                        latestImage.image,
                        "Ready to collect",
                        cv::Point(20, 40),
                        cv::FONT_HERSHEY_DUPLEX,
                        1.0,
                        cv::Scalar(255, 0, 0),
                        2);
                    break;
                }
                case CalibratorUtils::COLLECTING: {
                    // Get the latest observation
                    std::stringstream ss;
                    {
                        std::lock_guard<std::mutex> lock(targetObservationsMutex);
                        ss << "Collected " << camTargetObservations.at(0)->size() << " images";
                    }

                    cv::putText(
                        latestImage.image,
                        ss.str(),
                        cv::Point(20, 40),
                        cv::FONT_HERSHEY_DUPLEX,
                        1.0,
                        cv::Scalar(255, 0, 0),
                        2);
                    break;
                }
                case CalibratorUtils::COLLECTED: {
                    std::stringstream ss;
                    {
                        std::lock_guard<std::mutex> lock(targetObservationsMutex);
                        ss << "Ready to calibrate with " << camTargetObservations.at(0)->size() << " images";
                    }

                    cv::putText(
                        latestImage.image,
                        ss.str(),
                        cv::Point(20, 40),
                        cv::FONT_HERSHEY_DUPLEX,
                        1.0,
                        cv::Scalar(255, 0, 0),
                        2);
                    break;
                }
                case CalibratorUtils::CALIBRATING: {
                    cv::putText(
                        latestImage.image,
                        "Calibrating...",
                        cv::Point(20, 40),
                        cv::FONT_HERSHEY_DUPLEX,
                        1.0,
                        cv::Scalar(255, 0, 0),
                        2);
                    break;
                }
                case CalibratorUtils::CALIBRATED:
                    cv::putText(
                        latestImage.image,
                        "Calibrated!",
                        cv::Point(20, 40),
                        cv::FONT_HERSHEY_DUPLEX,
                        1.0,
                        cv::Scalar(255, 0, 0),
                        2);
                    break;
                default: throw std::runtime_error("Unknown state");
            }

            if (obs != nullptr) {
                // rounding between double and int64 timestamp can cause a small difference
                const auto timeDiff = std::abs(obs->time().toDvTime() - latestImage.timestamp);
                if (timeDiff <= 1) {
                    cv::Point prevPoint(-1, -1);
                    for (size_t y = 0; y < grid->rows(); ++y) {
                        const auto color = colors[y % colors.size()];
                        for (size_t x = 0; x < grid->cols(); ++x) {
                            const auto idx = y * grid->cols() + x;
                            Eigen::Vector2d point;
                            if (obs->imagePoint(idx, point)) {
                                const cv::Point cvPoint(static_cast<int>(point.x()), static_cast<int>(point.y()));
                                cv::circle(latestImage.image, cvPoint, 8, color, 2);
                                if (prevPoint != cv::Point(-1, -1)) {
                                    cv::line(latestImage.image, prevPoint, cvPoint, color, 2);
                                }
                                prevPoint = cvPoint;
                            }
                        }
                    }
                }
            }
            cameraId++;
        }
        return previews;
    }

    std::pair<cv::Matx33d, std::vector<double>> getCamMatrixDistortion(
        const boost::shared_ptr<CameraGeometry<CameraGeometryType, DistortionType>>& cam) {
        auto cam0Projection = cam->getDv()->projectionDesignVariable()->getParameters();
        auto cam0Dist = cam->getDv()->distortionDesignVariable()->getParameters();

        cv::Matx33d cam0Matrix = cv::Matx33d::eye();
        cam0Matrix.operator()(0, 0) = cam0Projection(0);
        cam0Matrix.operator()(1, 1) = cam0Projection(1);
        cam0Matrix.operator()(0, 2) = cam0Projection(2);
        cam0Matrix.operator()(1, 2) = cam0Projection(3);

        return std::make_pair(
            cam0Matrix,
            std::vector<double>({cam0Dist(0), cam0Dist(1), cam0Dist(2), cam0Dist(3), 0.0}));
    }

    /// Calculate average orientation using quaternions
    template<typename DataType, typename ForwardIterator>
    Eigen::Quaternion<DataType> averageQuaternions(ForwardIterator const& begin, ForwardIterator const& end) {
        if (begin == end) {
            throw std::logic_error("Cannot average orientations over an empty range.");
        }

        Eigen::Matrix<DataType, 4, 4> A = Eigen::Matrix<DataType, 4, 4>::Zero();
        uint sum(0);
        for (ForwardIterator it = begin; it != end; ++it) {
            Eigen::Matrix<DataType, 1, 4> q(1, 4);
            q(0) = it->w();
            q(1) = it->x();
            q(2) = it->y();
            q(3) = it->z();
            A += q.transpose() * q;
            sum++;
        }
        A /= sum;

        Eigen::EigenSolver<Eigen::Matrix<DataType, 4, 4>> es(A);

        Eigen::Matrix<std::complex<DataType>, 4, 1> mat(es.eigenvalues());
        int index;
        mat.real().maxCoeff(&index);
        Eigen::Matrix<DataType, 4, 1> largest_ev(es.eigenvectors().real().block(0, index, 4, 1));

        return Eigen::Quaternion<DataType>(largest_ev(0), largest_ev(1), largest_ev(2), largest_ev(3));
    }

    /// Overloaded function to calculate the average quaternion for some container types
    template<typename DataType, typename Container>
    typename Eigen::Quaternion<DataType> averageQuaternions(Container const& container) {
        return averageQuaternions<DataType>(container.begin(), container.end());
    }

    boost::shared_ptr<sm::kinematics::Transformation> estimateBaseline(
        const boost::shared_ptr<CameraGeometry<CameraGeometryType, DistortionType>>& c0,
        const boost::shared_ptr<CameraGeometry<CameraGeometryType, DistortionType>>& c1,
        const IccCameraUtils::ObservationsPtr& obs0,
        const IccCameraUtils::ObservationsPtr& obs1) {
        // For each observation
        dv::runtime_assert(obs0->size() == obs1->size(), "Number of observations in both camera should be the same");

        Eigen::Vector3d translationSum = Eigen::Vector3d::Zero();
        std::vector<Eigen::Quaterniond> quaternions;

        for (const auto& [time, cam0obs] : *obs0) {
            const auto& cam1obs = (*obs1)[time];

            if (!cam0obs.hasSuccessfulObservation() || !cam1obs.hasSuccessfulObservation()) {
                continue;
            }

            std::vector<cv::Point2f> pixelsCam0, pixelsCam1;
            cam0obs.getCornersImageFrame(pixelsCam0);
            cam1obs.getCornersImageFrame(pixelsCam1);
            std::vector<cv::Point3f> targetCam0, targetCam1;
            cam0obs.getCornersTargetFrame(targetCam0);
            cam1obs.getCornersTargetFrame(targetCam1);

            auto [cam0Mat, cam0Dist] = getCamMatrixDistortion(c0);
            auto [cam1Mat, cam1Dist] = getCamMatrixDistortion(c1);

            cv::Mat rvec0, tvec0, rvec1, tvec1;
            cv::solvePnP(targetCam0, pixelsCam0, cam0Mat, cam0Dist, rvec0, tvec0);
            cv::solvePnP(targetCam1, pixelsCam1, cam1Mat, cam1Dist, rvec1, tvec1);

            cv::Mat rotMat0, rotMat1;
            cv::Rodrigues(rvec0, rotMat0);
            cv::Rodrigues(rvec1, rotMat1);

            dv::kinematics::Transformationd T_t_c0(0, tvec0, rotMat0);
            dv::kinematics::Transformationd T_t_c1(0, tvec1, rotMat1);

            dv::kinematics::Transformationd T_c0_c1(0, T_t_c0.inverse().getTransform() * T_t_c1.getTransform());
            translationSum += T_c0_c1.getTranslation();
            quaternions.push_back(T_c0_c1.getQuaternion());
        }

        return boost::make_shared<sm::kinematics::Transformation>(
            averageQuaternions<double>(quaternions).coeffs(),
            translationSum / static_cast<double>(quaternions.size()));
    }

    /**
     * Calibrate the camera intrinsics (monocular).
     */
    std::optional<std::vector<CameraCalibrationUtils::CalibrationResult>> calibrateCameraIntrinsics() override {
        state = CalibratorUtils::CALIBRATING;

        static constexpr bool doBlakeZisserman = false;
        std::vector<boost::shared_ptr<CameraGeometry<CameraGeometryType, DistortionType>>> geometries;
        {
            size_t camId = 0;
            for (const auto& iccCamera : iccCameras) {
                auto& targetObservations = camTargetObservations.at(camId);
                std::cout << "Calibrating intrinsics using " << targetObservations->size() << " detections."
                          << std::endl;

                auto cameraGeometry = boost::make_shared<CameraGeometry<CameraGeometryType, DistortionType>>(iccCamera);
                if (!cameraGeometry->initGeometryFromObservations(targetObservations, grid)) {
                    throw std::runtime_error("Could not initialize the intrinsics");
                }
                geometries.push_back(cameraGeometry);
                camId++;
            }
        }

        std::vector<boost::shared_ptr<sm::kinematics::Transformation>> baselines;
        // Baseline c0->c0 is identity
        baselines.emplace_back(boost::make_shared<sm::kinematics::Transformation>(Eigen::Matrix4d::Identity()));

        // Initialize the baselines
        if (geometries.size() > 1) {
            for (auto nextCam = std::next(geometries.begin()); nextCam < geometries.end(); nextCam++) {
                auto prevCam = std::prev(nextCam);
                size_t prevCamId = std::distance(geometries.begin(), prevCam);
                size_t nextCamId = std::distance(geometries.begin(), nextCam);
                auto& prevObservations = camTargetObservations.at(prevCamId);
                auto& nextObservations = camTargetObservations.at(nextCamId);
                auto baseline = estimateBaseline(*prevCam, *nextCam, prevObservations, nextObservations);
                baselines.push_back(baseline);
            }
        }

        dv::runtime_assert(geometries.size() == iccCameras.size(), "Wrong camera count initialized for calibration");

        size_t removedOutlierCornersCount = 0u;
        bool initOutlierRejection = true;
        while (true) {
            try {
                std::cout << "calibrateIntrinsics: initializing calibrator" << std::endl;
                CameraCalibration<CameraGeometryType, DistortionType>
                    calibrator(geometries, grid, baselines, false, doBlakeZisserman);

                size_t view_id = 0;
                for (const auto& [timestamp, observation] : *camTargetObservations.at(0)) {
                    std::map<size_t, aslam::cameras::GridCalibrationTargetObservation> targetViews;
                    targetViews.emplace(0, observation);
                    for (const auto& [cameraId, observations] : camTargetObservations) {
                        if (cameraId > 0) {
                            targetViews.emplace(cameraId, observations->at(timestamp));
                        }
                    }
                    bool success = calibrator.addTargetView(targetViews);

                    //                static constexpr bool allowEndFiltering = true;
                    const bool runEndFiltering = (view_id == camTargetObservations.at(0)->size() - 1);
                    const auto numActiveBatches = calibrator.getNumBatches();
                    static size_t numCams = geometries.size();
                    static constexpr size_t minViewOutlier = 20;
                    static constexpr bool removeOutliers = true;
                    if (((success && numActiveBatches > minViewOutlier * numCams)
                         || (runEndFiltering && numActiveBatches > minViewOutlier * numCams))) {
                        // create the list of the batches to check
                        std::vector<size_t> batches_to_check;
                        if (initOutlierRejection) {
                            // check all views after the min. number of batches has been reached
                            for (size_t i = 0; i < calibrator.getNumBatches(); ++i) {
                                batches_to_check.push_back(i);
                            }
                            std::cout << "calibrateIntrinsics: Filtering outliers in all batches" << std::endl;
                            initOutlierRejection = false;
                        } else if (runEndFiltering) {
                            // check all batches again after all views have been processed
                            for (size_t i = 0; i < calibrator.getNumBatches(); ++i) {
                                batches_to_check.push_back(i);
                            }
                            std::cout << "calibrateIntrinsics: All views have been processed. Starting final outlier "
                                         "filtering..."
                                      << std::endl;
                        } else {
                            // only check most recent view
                            batches_to_check.push_back(calibrator.getNumBatches() - 1);
                        }
                        std::sort(batches_to_check.rbegin(), batches_to_check.rend());

                        for (const auto& batch_id : batches_to_check) {
                            // check all cameras in this batch
                            std::vector<std::vector<size_t>> cornerRemovalList_allCams;
                            // only one camera (mono) is supported at the moment

                            for (size_t camId = 0; camId < numCams; camId++) {
                                // calculate the reprojection errors statistics
                                const auto [corners, reprojs, rerrs] = calibrator.getReprojectionErrors(camId);
                                const auto [me, se] = getReprojectionErrorStatistics(rerrs);
                                const auto se_threshold = 4.0 * se;

                                // select corners to remove
                                std::vector<size_t> cornerRemovalList;
                                for (size_t pidx = 0; pidx < rerrs.at(batch_id).size(); ++pidx) {
                                    const auto reproj = rerrs.at(batch_id).at(pidx);
                                    if ((reproj.size() != 0)
                                        && (std::abs(reproj(0, 0)) > se_threshold.x()
                                            || std::abs(reproj(1, 0)) > se_threshold.y())) {
                                        cornerRemovalList.push_back(pidx);
                                        ++removedOutlierCornersCount;
                                    }
                                }

                                // queue corners on this cam for removal
                                cornerRemovalList_allCams.push_back(cornerRemovalList);
                            }
                            // we do not plot

                            // remove the corners (if there are corners to be removed)
                            size_t removeCount = 0;
                            for (const auto& list : cornerRemovalList_allCams) {
                                removeCount += list.size();
                            }

                            if (removeCount > 0) {
                                for (size_t camId = 0; camId < numCams; camId++) {
                                    auto new_batch = calibrator.removeCornersFromBatch(
                                        batch_id,
                                        camId,
                                        cornerRemovalList_allCams.at(camId),
                                        doBlakeZisserman);

                                    // replace the original batch with the corrected
                                    calibrator.replaceBatch(batch_id, new_batch);
                                }
                            }
                        }
                    }

                    ++view_id;
                }

                // final output

                size_t cameraId = 0;
                std::vector<CameraCalibrationUtils::CalibrationResult> results;
                for (const auto& iccCamera : iccCameras) {
                    std::cout << std::endl << "Intrinsics Calibration complete." << std::endl << std::endl;
                    std::cout << "Removed " << removedOutlierCornersCount << " outlier corners." << std::endl;
                    std::cout << "Processed " << camTargetObservations[cameraId]->size() << " images with "
                              << calibrator.getNumBatches() << " images used" << std::endl;
                    auto result = calibrator.getResult(cameraId);
                    CameraCalibrationUtils::printResult(result, std::cout);
                    std::cout << std::endl;

                    iccCamera->updateIntrinsics(result.projection, result.distortion);

                    if (cameraId > 0) {
                        // Update baseline using the new intrinsics
                        auto& prevObservations = camTargetObservations.at(cameraId - 1);
                        auto& nextObservations = camTargetObservations.at(cameraId);
                        auto baseline = estimateBaseline(
                            geometries.at(cameraId - 1),
                            geometries.at(cameraId),
                            prevObservations,
                            nextObservations);
                        result.baseline = baseline->T();
                    }

                    results.push_back(result);
                    cameraId++;
                }
                std::cout << "Finished calibration of intrinsics." << std::endl;

                return results;

            } catch (const OptimizationDiverged& exc) {
                std::cout
                    << "Optimization diverged possibly due to bad initialization. (Do the models fit the lenses well?)"
                    << std::endl;
                // not trying to restart
                break;
            }

            break; // Always break, restart on exception is not implemented
        }

        return std::nullopt;
    }

    /**
     * Build optimization problem. Needs to be called before calibrate().
     */
    void buildProblem() override {
        state = CalibratorUtils::CALIBRATING;

        if (camTargetObservations.at(0)->empty()) {
            throw std::runtime_error("No observations collected");
        }

        std::cout << "Calibrating using " << camTargetObservations.at(0)->size() << " detections." << std::endl;

        std::cout << std::endl << "Building the problem" << std::endl;
        iccCalibrator->buildProblem(
            6,
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
    }

    /**
     * Begin calibration procedure on the collected data.
     *
     * @return result of the calibration
     */
    [[nodiscard]] IccCalibratorUtils::CalibrationResult calibrate() override {
        std::lock_guard<std::mutex> lock1(targetObservationsMutex);
        std::lock_guard<std::mutex> lock2(imuDataMutex);

        std::stringstream ss;
        ss << std::endl << "Before Optimization" << std::endl << "###################" << std::endl;
        iccCalibrator->printErrorStatistics(ss);

        ss << std::endl << "Optimizing..." << std::endl;
        std::cout << ss.str();
        ss.str("");
        iccCalibrator->optimize(nullptr, calibratorOptions.maxIter, false);

        ss << std::endl << "After Optimization" << std::endl << "###################" << std::endl;
        iccCalibrator->printErrorStatistics(ss);

        ss << std::endl << "Results" << std::endl << "#######" << std::endl << std::endl;
        auto result = iccCalibrator->getResult();
        IccCalibratorUtils::printResult(result, ss);
        std::cout << ss.str();

        state = CalibratorUtils::CALIBRATED;

        return result;
    }

    /**
     * Start collecting data from the camera and IMU.
     */
    void startCollecting() override {
        reset();

        state = CalibratorUtils::COLLECTING;
    }

    /**
     * Stop collecting data from the camera and IMU.
     */
    void stopCollecting() override {
        std::cout << "Waiting for the detector to finish..." << std::endl;
        detectionsQueue.waitForEmptyQueue();

        state = CalibratorUtils::COLLECTED;
    }

    /**
     * Discard the collected data and reset the calibrator.
     */
    void reset() override {
        detectionsQueue.waitForEmptyQueue();

        {
            std::lock_guard<std::mutex> lock1(targetObservationsMutex);
            for (auto& [_, targetObservations] : camTargetObservations) {
                targetObservations->clear();
            }
        }

        {
            std::lock_guard<std::mutex> lock2(imuDataMutex);
            imuData->clear();
        }
    }

    /**
     * Get a string containing DV log information before optimization.
     *
     * @param ss string stream into which log will be output
     */
    void getDvInfoBeforeOptimization(std::ostream& ss) override {
        ss << "Calibrating using ";
        {
            std::lock_guard<std::mutex> lock1(targetObservationsMutex);
            ss << camTargetObservations.at(0)->size() << " target observations and ";
        }

        {
            std::lock_guard<std::mutex> lock2(imuDataMutex);
            ss << imuData->size() << " IMU measurements " << std::endl;
        }

        ss << "BEFORE OPTIMIZATION" << std::endl;
        iccCalibrator->printErrorStatistics(ss);
    }

    /**
     * Get a string containing DV log information after optimization.
     *
     * @param ss string stream into which log will be output
     */
    void getDvInfoAfterOptimization(std::ostream& ss) override {
        ss << "AFTER OPTIMIZATION" << std::endl;
        iccCalibrator->printErrorStatistics(ss);
    }

protected:
    /**
     * Detect the calibration pattern on the given stamped image.
     *
     * @param stampedImage
     */
    void detectPattern(const std::vector<CalibratorUtils::StampedImage>& frames) override {
        dv::runtime_assert(frames.size() == iccCameras.size(), "Wrong number of frames passed for detection");

        std::vector<aslam::cameras::GridCalibrationTargetObservation> observations;
        std::vector<bool> successes;
        observations.resize(frames.size());
        successes.resize(frames.size());

        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, frames.size()),
            [this, &observations, &frames, &successes](const auto& range) {
                const size_t cameraId = range.begin();
                const auto& stampedImage = frames[cameraId];
                const auto& detector = detectors[cameraId];
                auto& observation = observations[cameraId];

                // Search for pattern and draw it on the image frame
                if (detector->findTarget(
                        stampedImage.image,
                        aslam::Time(IccSensors::toSec(stampedImage.timestamp)),
                        observation)) {
                    successes[cameraId] = observation.hasSuccessfulObservation();
                } else {
                    successes[cameraId] = false;
                }
            });

        // All are true
        bool success = std::all_of(successes.begin(), successes.end(), [](const auto& a) {
            return a;
        });

        // If pattern detected add it to observation
        if (state == CalibratorUtils::COLLECTING && success) {
            std::lock_guard<std::mutex> lock(targetObservationsMutex);
            size_t cameraId = 0;
            for (auto& observation : observations) {
                camTargetObservations[cameraId]->emplace(frames[cameraId].timestamp, observation);
                cameraId++;
            }
        }

        // Replace the most recent image even if no pattern detected
        std::lock_guard<std::mutex> lock1(latestImageMutex);
        latestImages = frames;
        if (success) {
            latestObservations.clear();
            for (auto& observation : observations) {
                latestObservations.push_back(
                    boost::make_shared<aslam::cameras::GridCalibrationTargetObservation>(observation));
            }
        }
    }
};
