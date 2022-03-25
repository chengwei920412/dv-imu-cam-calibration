#include <utilities/calibrator.hpp>

#include <aslam/cameras.hpp>

#include <dv-sdk/module.hpp>

#include <fmt/chrono.h>
#include <regex>

bool cvExists(const cv::FileNode& fn) {
    if (fn.type() == cv::FileNode::NONE) {
        return (false);
    }

    return (true);
}

class ImuCamCalibration : public dv::ModuleBase {
protected:
    // Calibrator options
    CalibratorUtils::Options mOptions;

    std::unique_ptr<CalibratorBase> calibrator = nullptr;

    enum CollectionState { BEFORE_COLLECTING, DURING_COLLECTING, AFTER_COLLECTING, CALIBRATED };

    CollectionState collectionState = BEFORE_COLLECTING;

public:
    static void initInputs(dv::InputDefinitionList& in) {
        in.addFrameInput("frames");
        in.addIMUInput("imu");
    }

    static void initOutputs(dv::OutputDefinitionList& out) {
        out.addFrameOutput("preview");
    }

    static const char* initDescription() {
        return ("Calibrate the IMU with respect to the event camera.");
    }

    static void initConfigOptions(dv::RuntimeConfig& config) {
        // Camera calibration file output
        config.add(
            "outputCalibrationDirectory",
            dv::ConfigOption::directoryOption(
                "Specify directory to save the calibration settings in",
                dv::portable_get_user_home_directory()));

        // Calibration pattern
        config.add("boardHeight", dv::ConfigOption::intOption("Number of rows in the calibration pattern", 11, 1, 50));
        config.add("boardWidth", dv::ConfigOption::intOption("Number of cols in the calibration pattern", 4, 1, 50));
        config.add(
            "boardSquareSize",
            dv::ConfigOption::doubleOption("Size of a calibration pattern element in meters", 0.05, 0.0, 1.0));
        config.add(
            "tagSpacingRatio",
            dv::ConfigOption::doubleOption("Ratio of space between tags to tagSize (AprilTag)", 0.3, 0.0, 1.0));
        config.add(
            "calibrationPattern",
            dv::ConfigOption::listOption(
                "Calibration pattern to use",
                "asymmetricCirclesGrid",
                {"chessboard", "asymmetricCirclesGrid", "aprilTag"},
                false));

        config.add(
            "calibrationModel",
            dv::ConfigOption::listOption(
                "Calibration model to use",
                "Pinhole-RadialTangential",
                {"Pinhole-RadialTangential", "Pinhole-Equidistant", "Pinhole-Fov"},
                false));

        // Module control buttons
        config.add(
            "startCollecting",
            dv::ConfigOption::buttonOption("Start collecting calibration images", "StartCollecting"));
        config.add(
            "stopCollecting",
            dv::ConfigOption::buttonOption("Stop collecting calibration images", "StopCollecting"));
        config.add("discard", dv::ConfigOption::buttonOption("Discard the collected images", "Discard"));
        config.add("calibrate", dv::ConfigOption::buttonOption("Start calibration algorithm", "Calibrate"));

        // Optimization options
        config.add(
            "maxIter",
            dv::ConfigOption::intOption("Maximum number of iteration of calibration optimization problem", 50, 1, 100));
        config.add(
            "timeCalibration",
            dv::ConfigOption::boolOption("If true, time offset between the sensors will be calibrated", true));

        // IMU noise parameters
        config.add("IMUupdateRate", dv::ConfigOption::doubleOption("IMU update rate [Hz]", 200, 10, 10000));
        config.add("accNoiseDensity", dv::ConfigOption::doubleOption("Accelerometer noise density", 1.49e-3, 0, 1));
        config.add("accRandomWalk", dv::ConfigOption::doubleOption("Accelerometer noise random walk", 8.69e-5, 0, 1));
        config.add("gyrNoiseDensity", dv::ConfigOption::doubleOption("Gyroscope noise density", 8.09e-5, 0, 1));
        config.add("gyrRandomWalk", dv::ConfigOption::doubleOption("Gyroscope noise random walk", 2.29e-6, 0, 1));

        // Optimization options
        config.add(
            "maxIter",
            dv::ConfigOption::intOption("Maximum number of iteration of calibration optimization problem", 50, 1, 100));
        config.add(
            "timeCalibration",
            dv::ConfigOption::boolOption("If true, time offset between the sensors will be calibrated", true));

        config.setPriorityOptions(
            {"outputCalibrationDirectory",
             "boardHeight",
             "boardWidth",
             "boardSquareSize",
             "calibrationPattern",
             "startCollecting",
             "stopCollecting",
             "discard",
             "calibrate"});
    }

    void configUpdate() {
        // Handle user input
        switch (collectionState) {
            case BEFORE_COLLECTING: {
                setupCalibrator();
                if (config.getBool("startCollecting")) {
                    collectionState = DURING_COLLECTING;
                    calibrator->startCollecting();
                    log.info("Started collecting images");
                }
                break;
            }
            case DURING_COLLECTING: {
                if (calibrator == nullptr) {
                    throw std::runtime_error("Calibrator is not initialized.");
                }
                if (config.getBool("stopCollecting")) {
                    collectionState = AFTER_COLLECTING;
                    calibrator->stopCollecting();
                    log.info("Stopped collecting images");
                }
                break;
            }
            case AFTER_COLLECTING: {
                if (calibrator == nullptr) {
                    throw std::runtime_error("Calibrator is not initialized.");
                }
                if (config.getBool("calibrate")) {
                    calibrate();
                    collectionState = CALIBRATED;
                }

                if (config.getBool("discard")) {
                    collectionState = BEFORE_COLLECTING;
                    log.info("Discarded all collected data");
                    initializeCalibrator();
                }
                break;
            }
            case CALIBRATED: {
                if (calibrator == nullptr) {
                    throw std::runtime_error("Calibrator is not initialized.");
                }
                if (config.getBool("discard")) {
                    collectionState = BEFORE_COLLECTING;
                    log.info("Discarded all collected data");
                    initializeCalibrator();
                }
                break;
            }
            default: throw std::runtime_error("Invalid collection state");
        }

        // Enable/Disable buttons based on current state
        switch (collectionState) {
            case BEFORE_COLLECTING: {
                config.setBool("startCollecting", false);
                config.setBool("stopCollecting", true);
                config.setBool("discard", true);
                config.setBool("calibrate", true);
                config.setBool("calibrationModel", true);
                break;
            }
            case DURING_COLLECTING: {
                config.setBool("startCollecting", true);
                config.setBool("stopCollecting", false);
                config.setBool("discard", true);
                config.setBool("calibrate", true);
                config.setBool("calibrationModel", false);
                break;
            }
            case AFTER_COLLECTING: {
                config.setBool("startCollecting", true);
                config.setBool("stopCollecting", true);
                config.setBool("discard", false);
                config.setBool("calibrate", false);
                config.setBool("calibrationModel", false);
                break;
            }
            case CALIBRATED: {
                config.setBool("startCollecting", true);
                config.setBool("stopCollecting", true);
                config.setBool("discard", false);
                config.setBool("calibrate", true);
                config.setBool("calibrationModel", true);
                break;
            }
            default: throw std::runtime_error("Invalid collection state");
        }
    }

    void setupCalibrator() {
        if (calibrator != nullptr) {
            calibrator->reset();
        }
        if (config.getString("calibrationModel") == "Pinhole-Equidistant") {
            calibrator = std::make_unique<Calibrator<
                aslam::cameras::EquidistantDistortedPinholeCameraGeometry,
                aslam::cameras::EquidistantDistortion>>(mOptions);
        } else if (config.getString("calibrationModel") == "Pinhole-Fov") {
            calibrator = std::make_unique<
                Calibrator<aslam::cameras::FovDistortedPinholeCameraGeometry, aslam::cameras::FovDistortion>>(mOptions);
        } else {
            calibrator = std::make_unique<
                Calibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>>(
                mOptions);
        }
    }

    void initializeCalibrator() {
        const auto frameInput = inputs.getFrameInput("frames");
        const auto description = frameInput.getOriginDescription();

        mOptions.pattern = getCalibrationPattern();
        mOptions.rows = static_cast<size_t>(config.getInt("boardHeight"));
        mOptions.cols = static_cast<size_t>(config.getInt("boardWidth"));
        mOptions.spacingMeters = config.getDouble("boardSquareSize");
        mOptions.tagSpacing = config.getDouble("tagSpacingRatio");
        mOptions.imageSize = frameInput.size();

        mOptions.maxIter = static_cast<size_t>(config.getInt("maxIter"));
        mOptions.timeCalibration = config.getBool("timeCalibration");
    }

    ImuCamCalibration() {
        // Input output
        const auto frameInput = inputs.getFrameInput("frames");
        const auto inputSize = frameInput.size();
        const auto description = frameInput.getOriginDescription();
        outputs.getFrameOutput("preview").setup(inputSize.width, inputSize.height, description);

        initializeCalibrator();

        // Unclick all the buttons and update the state
        config.setBool("startCollecting", false);
        config.setBool("stopCollecting", false);
        config.setBool("discard", false);
        config.setBool("calibrate", false);
        configUpdate();
    }

    void run() override {
        // Process IMU input
        auto imuInput = inputs.getIMUInput("imu");
        if (auto imuData = imuInput.data()) {
            for (const auto& singleImu : imuData) {
                calibrator->addImu(
                    singleImu.timestamp,
                    static_cast<double>(singleImu.gyroscopeX) * M_PI / 180.0,
                    static_cast<double>(singleImu.gyroscopeY) * M_PI / 180.0,
                    static_cast<double>(singleImu.gyroscopeZ) * M_PI / 180.0,
                    static_cast<double>(singleImu.accelerometerX) * 9.81,
                    static_cast<double>(singleImu.accelerometerY) * 9.81,
                    static_cast<double>(singleImu.accelerometerZ) * 9.81);
            }
        }

        // Process frame input
        auto frameInput = inputs.getFrameInput("frames");
        if (auto frame = frameInput.data()) {
            cv::Mat img = *frame.getMatPointer();
            calibrator->addImage(img, frame.timestamp());

            // Output preview image
            auto preview = calibrator->getPreviewImage();
            outputs.getFrameOutput("preview") << preview.timestamp << preview.image << dv::commit;
        }
    }

protected:
    std::string saveFilePath() {
        const auto frameInput = inputs.getFrameInput("frames");
        const auto cameraID = frameInput.getOriginDescription();
        boost::filesystem::path outputDir{config.getString("outputCalibrationDirectory")};

        if (outputDir.empty()) {
            outputDir = dv::portable_get_user_home_directory();
        }

        const std::string timeString = fmt::format(
            "{:%Y_%m_%d_%H_%M_%S}",
            fmt::localtime(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())));
        const std::string fileName = "camera_imu_calibration_" + cameraID + "-" + timeString + ".xml";

        auto filePath = (outputDir / fileName).string();
        return filePath;
    }

    CalibratorUtils::CalibrationPattern getCalibrationPattern() {
        const auto pattern = config.getString("calibrationPattern");
        if (pattern == "chessboard") {
            return CalibratorUtils::CalibrationPattern::CHESSBOARD;
        } else if (pattern == "asymmetricCirclesGrid") {
            return CalibratorUtils::CalibrationPattern::ASYMMETRIC_CIRCLES_GRID;
        } else if (pattern == "aprilTag") {
            return CalibratorUtils::CalibrationPattern::APRIL_GRID;
        } else {
            std::stringstream ss;
            ss << "Unknown calibration pattern: " << pattern;
            throw std::runtime_error(ss.str());
        }
    }

    cv::Size getBoardSize() {
        switch (getCalibrationPattern()) {
            case CalibratorUtils::CalibrationPattern::CHESSBOARD:
                // Inner corners, so -1 on each side.
                return cv::Size(config.getInt("boardWidth") - 1, config.getInt("boardHeight") - 1);
            case CalibratorUtils::CalibrationPattern::APRIL_GRID:
            case CalibratorUtils::CalibrationPattern::ASYMMETRIC_CIRCLES_GRID:
                return cv::Size(config.getInt("boardWidth"), config.getInt("boardHeight"));
            default: {
                std::stringstream ss;
                ss << "Can not get the board size for unknown calibration pattern: "
                   << config.getString("calibrationPattern");
                throw std::runtime_error(ss.str());
            }
        }
    }

    void saveCalibration(
        const CameraCalibrationUtils::CalibrationResult& intrinsicResult,
        const IccCalibratorUtils::CalibrationResult& result) {
        const auto filePath = saveFilePath();

        cv::FileStorage fs(filePath, cv::FileStorage::WRITE);

        if (!fs.isOpened()) {
            log.warning << "Failed to write to calibration file: " << filePath << dv::logEnd;
            return;
        }

        // Camera matrix in OpenCV format
        std::vector<double> camMatValues{
            intrinsicResult.projection.at(0),
            0.0,
            intrinsicResult.projection.at(2),
            0.0,
            intrinsicResult.projection.at(1),
            intrinsicResult.projection.at(3),
            0.0,
            0.0,
            1.0};
        cv::Mat cameraMatrix(3, 3, CV_64FC1, camMatValues.data());

        // Distortion coefficients in OpenCV format
        std::vector<double> distCopy(intrinsicResult.distortion);
        cv::Mat distCoeffs(1, static_cast<int>(intrinsicResult.distortion.size()), CV_64FC1, distCopy.data());

        // Get the camera ID
        const auto frameInput = inputs.getFrameInput("frames");
        const auto originDescription = frameInput.getOriginDescription();
        static const std::regex filenameCleanupRegex{"[^a-zA-Z-_\\d]"};
        auto cameraID = std::regex_replace(originDescription, filenameCleanupRegex, "_");
        if ((cameraID[0] == '-') || (std::isdigit(cameraID[0]) != 0)) {
            cameraID = "_" + cameraID;
        }

        // Single camera calibration data
        fs << cameraID; // Node name.
        fs << "{";      // Node start.
        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distCoeffs;
        fs << "image_width" << frameInput.sizeX();
        fs << "image_height" << frameInput.sizeY();
        fs << "}"; // Node end.
        fs << "use_fisheye_model" << false;
        fs << "type"
           << "camera";

        // The fields in the block below existed in DV Camera Calibration module, for compatibility reasons they are
        // also output here
        fs << "pattern_width" << getBoardSize().width;
        fs << "pattern_height" << getBoardSize().height;
        if (config.getString("calibrationModel") == "Pinhole-Equidistant") {
            fs << "distortion_model"
               << "Equidistant";
        } else if (config.getString("calibrationModel") == "Pinhole-Fov") {
            fs << "distortion_model"
               << "FOV";
        } else {
            fs << "distortion_model"
               << "RadialTangential";
        }
        fs << "pattern_type" << config.getString("calibrationPattern");
        fs << "board_width" << config.getInt("boardWidth");
        fs << "board_height" << config.getInt("boardHeight");
        fs << "square_size" << config.getFloat("boardSquareSize");
        fs << "calibration_error"
           << "N/A"; // Now we have two errors, intrinsic and extrinsic. This one is not output
                     // to avoid confusion
        fs << "calibration_time"
           << fmt::format(
                  "{:%c}",
                  fmt::localtime(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())));

        // The fields in the block below are related to camera IMU calibration. They did not exist in original camera
        // calibration module
        fs << "calibration_converged" << result.converged;
        fs << "time_offset_cam_imu__t_imu_equals_t_cam_plus_shift" << result.t_cam_imu;
        cv::Mat trans;
        cv::eigen2cv(result.T_cam_imu, trans);
        fs << "transformation_cam_imu" << trans;
        fs << "mean_reprojection_error" << result.error_info.meanReprojectionError;
        fs << "mean_accelerometer_error" << result.error_info.meanAccelerometerError;
        fs << "mean_gyroscope_error" << result.error_info.meanGyroscopeError;

        // Intrinsic calibration error
        cv::Mat intrErr;
        cv::eigen2cv(intrinsicResult.err_info.mean, intrErr);
        fs << "intrinsic_calibration_reprojection_error" << intrErr;

        log.info << "Calibration saved to a file: " << filePath << dv::logEnd;
    }

    void calibrate() {
        // Function printing std string to DV log with nicely handled newlines
        auto string2dvLog = [&](const std::string& str) {
            std::stringstream stringReader(str);
            std::string line;

            while (std::getline(stringReader, line)) {
                if (!line.empty()) {
                    log.info << line << dv::logEnd;
                }
            }
        };

        std::stringstream ss;

        log.info("Calibrating the intrinsics of the camera...");
        auto intrinsicsResult = calibrator->calibrateCameraIntrinsics();
        if (!intrinsicsResult.has_value()) {
            throw std::runtime_error("Failed to calibrate intrinsics! Please check the dataset");
        }
        CameraCalibrationUtils::printResult(intrinsicsResult.value(), ss);
        string2dvLog(ss.str());
        ss.str("");

        log.info("Building the problem...");
        calibrator->buildProblem();

        // Print the info before optimization
        calibrator->getDvInfoBeforeOptimization(ss);
        string2dvLog(ss.str());
        ss.str("");

        // Run the optimization problem
        log.info("Optimizing...");
        const auto result = calibrator->calibrate();

        // Print the info after optimization
        calibrator->getDvInfoAfterOptimization(ss);
        string2dvLog(ss.str());
        ss.str("");

        // Print the result
        log.info("RESULT");
        IccCalibratorUtils::printResult(result, ss);
        string2dvLog(ss.str());

        // Save the calibration to a text file
        saveCalibration(intrinsicsResult.value(), result);
    }
};

registerModuleClass(ImuCamCalibration)
