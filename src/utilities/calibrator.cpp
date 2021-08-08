//
// Created by radam on 2021-03-23.
//

#include <utilities/calibrator.hpp>

#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <kalibr_calibrate_cameras/CameraCalibrator.hpp>
#include <sm/boost/JobQueue.hpp>

#include <Eigen/Eigen>

Calibrator::StampedImage previewImageWithText(
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

    return Calibrator::StampedImage(img, timestamp);
}

Calibrator::Calibrator(const Options& opts) : calibratorOptions(opts) {
    if (calibratorOptions.imageSize.area() == 0) {
        throw std::runtime_error("Image size given to the calibrator is incorrect. The area is equal zero.");
    }

    // Guess the intrinsics of the camera
    if (calibratorOptions.intrinsics.empty()) {
        calibratorOptions.intrinsics.push_back(calibratorOptions.imageSize.width);
        calibratorOptions.intrinsics.push_back(calibratorOptions.imageSize.width);
        calibratorOptions.intrinsics.push_back(calibratorOptions.imageSize.width / 2);
        calibratorOptions.intrinsics.push_back(calibratorOptions.imageSize.height / 2);
    }

    // Guess the distortion coefficients
    if (calibratorOptions.distCoeffs.empty()) {
        calibratorOptions.distCoeffs.push_back(0.0);
        calibratorOptions.distCoeffs.push_back(0.0);
        calibratorOptions.distCoeffs.push_back(0.0);
        calibratorOptions.distCoeffs.push_back(0.0);
    }

    targetObservations = boost::make_shared<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>>();
    imuData = boost::make_shared<std::vector<ImuMeasurement>>();

    iccImu = boost::make_shared<IccImu>(calibratorOptions.imuParameters, imuData);
    iccCamera = boost::make_shared<IccCamera>(
        calibratorOptions.intrinsics,
        calibratorOptions.distCoeffs,
        calibratorOptions.imageSize,
        targetObservations);
    iccCalibrator = boost::make_shared<IccCalibrator>(iccCamera, iccImu);

    detectionsQueue.start(std::max(1u, std::thread::hardware_concurrency() - 1));

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

            grid = boost::make_shared<aslam::cameras::GridCalibrationTargetCheckerboard>(
                calibratorOptions.rows,
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
        case CalibrationPattern::APRIL_GRID: {
            std::cout << "  Type: APRIL_GRID" << std::endl;
            std::cout << "  Tags:" << std::endl;
            std::cout << "    Cols: " << calibratorOptions.cols << std::endl;
            std::cout << "    Rows: " << calibratorOptions.rows << std::endl;
            std::cout << "    Size: " << calibratorOptions.spacingMeters << " [m]" << std::endl;
            std::cout << "    Spacing : " << calibratorOptions.spacingMeters * calibratorOptions.tagSpacing << " [m]"
                      << std::endl;

            aslam::cameras::GridCalibrationTargetAprilgrid::AprilgridOptions options;
            // enforce more than one row --> pnp solution can be bad if all points are almost on a line...
            options.minTagsForValidObs = std::max(calibratorOptions.rows, calibratorOptions.cols) + 1;
            options.showExtractionVideo = false;

            grid = boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(
                calibratorOptions.rows,
                calibratorOptions.cols,
                calibratorOptions.spacingMeters,
                calibratorOptions.tagSpacing,
                options);
            break;
        }
        default: throw std::runtime_error("Not implemented calibration pattern");
    }

    detectorOptions.imageStepping = false;
    detectorOptions.plotCornerReprojection = false;
    detectorOptions.filterCornerOutliers = true;

    state = INITIALIZED;
}

Calibrator::~Calibrator() {
    detectionsQueue.join();
}

void Calibrator::addImu(
    const int64_t timestamp,
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

    if (state == COLLECTING) {
        boost::unique_future<void> job;
        detectionsQueue.scheduleFuture<void>(boost::bind(&Calibrator::detectPattern, this, stampedImage.clone()), job);
    } else {
        std::lock_guard<std::mutex> lock(latestImageMutex);
        latestStampedImage = boost::make_shared<StampedImage>(stampedImage.clone());
        latestObservation = nullptr;
    }
}

void Calibrator::addImage(const cv::Mat& img, int64_t timestamp) {
    addImage(StampedImage(img, timestamp));
}

Calibrator::StampedImage Calibrator::getPreviewImage() {
    // Get the latest image
    StampedImage latestImage;
    std::shared_ptr<aslam::cameras::GridCalibrationTargetObservation> latestObs = nullptr;
    {
        std::lock_guard<std::mutex> lock(latestImageMutex);
        if (latestStampedImage == nullptr) {
            return StampedImage(previewImageWithText("No image arrived yet"));
        }
        latestImage = latestStampedImage->clone();
        if (latestImage.image.channels() == 1) {
            cv::cvtColor(latestImage.image, latestImage.image, cv::COLOR_GRAY2BGR);
        }
        if (latestObservation != nullptr) {
            latestObs = std::make_shared<aslam::cameras::GridCalibrationTargetObservation>(*latestObservation);
        }
    }

    switch (state) {
        case INITIALIZED: {
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
        case COLLECTING: {
            // Get the latest observation
            std::stringstream ss;
            {
                std::lock_guard<std::mutex> lock(targetObservationsMutex);
                ss << "Collected " << targetObservations->size() << " images";
                auto it = targetObservations->find(latestImage.timestamp);
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
        case COLLECTED: {
            std::stringstream ss;
            {
                std::lock_guard<std::mutex> lock(targetObservationsMutex);
                ss << "Ready to calibrate with " << targetObservations->size() << " images";
                auto it = targetObservations->find(latestImage.timestamp);
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
        case CALIBRATING: {
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
        case CALIBRATED:
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

    if (latestObs != nullptr) {
        // rounding between double and int64 timestamp can cause a small difference
        const auto timeDiff = std::abs(latestObs->time().toDvTime() - latestImage.timestamp);
        if (timeDiff <= 1) {
            cv::Point prevPoint(-1, -1);
            for (size_t y = 0; y < grid->rows(); ++y) {
                const auto color = colors[y % colors.size()];
                for (size_t x = 0; x < grid->cols(); ++x) {
                    const auto idx = y * grid->cols() + x;
                    Eigen::Vector2d point;
                    if (latestObs->imagePoint(idx, point)) {
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

    return latestImage;
}

std::optional<CameraCalibration::CalibrationResult> Calibrator::calibrateCameraIntrinsics() {
    state = CALIBRATING;

    std::cout << "Calibrating intrinsics using " << targetObservations->size() << " detections." << std::endl;

    static constexpr bool doBlakeZisserman = false;

    auto cameraGeometry = boost::make_shared<CameraGeometry>(iccCamera);
    if (!cameraGeometry->initGeometryFromObservations(targetObservations, grid)) {
        throw std::runtime_error("Could not initialize the intrinsics");
    }

    size_t removedOutlierCornersCount = 0u;
    static constexpr bool removeOutliers = true;
    bool initOutlierRejection = true;
    while (true) {
        try {
            std::cout << "calibrateIntrinsics: initializing calibrator" << std::endl;
            CameraCalibration calibrator(cameraGeometry, grid, false, doBlakeZisserman);

            size_t view_id = 0;
            for (const auto& [timestamp, observation] : *targetObservations) {
                bool success = calibrator.addTargetView(observation);

                static constexpr bool allowEndFiltering = true;
                const bool runEndFiltering = (view_id == targetObservations->size() - 1) && (allowEndFiltering);
                const auto numActiveBatches = calibrator.getNumBatches();
                static constexpr size_t numCams = 1;
                static constexpr size_t minViewOutlier = 20;
                static constexpr bool removeOutliers = true;
                if (((success && numActiveBatches > minViewOutlier * numCams)
                     || (runEndFiltering && numActiveBatches > minViewOutlier * numCams))
                    and removeOutliers) {
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
                        std::cout
                            << "calibrateIntrinsics: All views have been processed. Starting final outlier filtering..."
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

                        // calculate the reprojection errors statistics
                        const auto [corners, reprojs, rerrs] = calibrator.getReprojectionErrors(0);
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

                        // we do not plot

                        // remove the corners (if there are corners to be removed)
                        size_t removeCount = 0;
                        for (const auto& list : cornerRemovalList_allCams) {
                            removeCount += list.size();
                        }

                        if (removeCount > 0) {
                            static constexpr size_t camId = 0;
                            auto new_batch = calibrator.removeCornersFromBatch(
                                batch_id,
                                cornerRemovalList_allCams.at(camId),
                                doBlakeZisserman);

                            // replace the original batch with the corrected
                            calibrator.replaceBatch(batch_id, new_batch);
                        }
                    }
                }

                ++view_id;
            }

            // final output

            std::cout << std::endl << "Intrinsics Calibration complete." << std::endl << std::endl;
            if (removeOutliers) {
                std::cout << "Removed " << removedOutlierCornersCount << " outlier corners." << std::endl;
            }
            std::cout << "Processed " << targetObservations->size() << " images with " << calibrator.getNumBatches()
                      << " images used" << std::endl;
            const auto result = calibrator.getResult();
            std::stringstream ss;
            calibrator.printResult(result, ss);
            std::cout << ss.str() << std::endl;

            iccCamera->updateIntrinsics(result.projection, result.distortion);
            std::cout << "Finished calibration of intrinsics." << std::endl;

            return result;

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

void Calibrator::buildProblem() {
    state = CALIBRATING;

    if (targetObservations->empty()) {
        throw std::runtime_error("No observations collected");
    }

    std::cout << "Calibrating using " << targetObservations->size() << " detections." << std::endl;

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

IccCalibrator::CalibrationResult Calibrator::calibrate() {
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
    IccCalibrator::printResult(result, ss);
    std::cout << ss.str();

    state = CALIBRATED;

    return result;
}

void Calibrator::detectPattern(const StampedImage& stampedImage) {
    std::vector<cv::Point2f> pointBuf;

    // Make a copy of the detector in each thread to avoid memory issues
    auto detector
        = boost::make_shared<aslam::cameras::GridDetector>(iccCamera->getCameraGeometry(), grid, detectorOptions);

    // Search for pattern and draw it on the image frame
    aslam::cameras::GridCalibrationTargetObservation observation;
    bool success = detector->findTarget(stampedImage.image, aslam::Time(toSec(stampedImage.timestamp)), observation);

    // If pattern detected add it to observation
    if (state == COLLECTING && success && observation.hasSuccessfulObservation()) {
        std::lock_guard<std::mutex> lock(targetObservationsMutex);
        targetObservations->emplace(stampedImage.timestamp, observation);
    }

    // Replace the most recent image even if no pattern detected
    std::lock_guard<std::mutex> lock1(latestImageMutex);
    bool replace = true;
    if (latestStampedImage != nullptr) {
        if (stampedImage.timestamp < latestStampedImage->timestamp) {
            replace = false;
        }
    }
    if (replace) {
        latestStampedImage = boost::make_shared<StampedImage>(stampedImage.clone());
        if (success) {
            latestObservation = boost::make_shared<aslam::cameras::GridCalibrationTargetObservation>(observation);
        }
    }
}

void Calibrator::startCollecting() {
    reset();

    state = COLLECTING;
}

void Calibrator::stopCollecting() {
    std::cout << "Waiting for the detector to finish..." << std::endl;
    detectionsQueue.waitForEmptyQueue();

    state = COLLECTED;
}

void Calibrator::reset() {
    detectionsQueue.waitForEmptyQueue();

    {
        std::lock_guard<std::mutex> lock1(targetObservationsMutex);
        targetObservations->clear();
    }

    {
        std::lock_guard<std::mutex> lock2(imuDataMutex);
        imuData->clear();
    }
}