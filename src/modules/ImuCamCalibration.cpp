#include <utilities/calibrator.hpp>

#include <dv-sdk/module.hpp>

#include <fmt/chrono.h>

bool cvExists(const cv::FileNode& fn) {
    if (fn.type() == cv::FileNode::NONE) {
        return (false);
    }

    return (true);
}

class ImuCamCalibration : public dv::ModuleBase {
protected:
    std::unique_ptr<Calibrator> calibrator = nullptr;

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
                if (config.getBool("startCollecting")) {
                    collectionState = DURING_COLLECTING;
                    calibrator->startCollecting();
                    log.info("Started collecting images");
                }
                break;
            }
            case DURING_COLLECTING: {
                if (config.getBool("stopCollecting")) {
                    collectionState = AFTER_COLLECTING;
                    calibrator->stopCollecting();
                    log.info("Stopped collecting images");
                }
                break;
            }
            case AFTER_COLLECTING: {
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
                break;
            }
            case DURING_COLLECTING: {
                config.setBool("startCollecting", true);
                config.setBool("stopCollecting", false);
                config.setBool("discard", true);
                config.setBool("calibrate", true);
                break;
            }
            case AFTER_COLLECTING: {
                config.setBool("startCollecting", true);
                config.setBool("stopCollecting", true);
                config.setBool("discard", false);
                config.setBool("calibrate", false);
                break;
            }
            case CALIBRATED: {
                config.setBool("startCollecting", true);
                config.setBool("stopCollecting", true);
                config.setBool("discard", false);
                config.setBool("calibrate", true);
                break;
            }
            default: throw std::runtime_error("Invalid collection state");
        }
    }

    void initializeCalibrator() {
        const auto frameInput = inputs.getFrameInput("frames");
        const auto description = frameInput.getOriginDescription();

        // Calibrator options
        Calibrator::Options options;

        // Calibration pattern
        const auto pattern = config.getString("calibrationPattern");
        if (pattern == "chessboard") {
            options.pattern = Calibrator::CalibrationPattern::CHESSBOARD;
        } else if (pattern == "asymmetricCirclesGrid") {
            options.pattern = Calibrator::CalibrationPattern::ASYMMETRIC_CIRCLES_GRID;
        } else if (pattern == "aprilTag") {
            options.pattern = Calibrator::CalibrationPattern::APRIL_GRID;
        } else {
            std::stringstream ss;
            ss << "Unknown calibration pattern: " << pattern;
            throw std::runtime_error(ss.str());
        }

        options.rows = static_cast<size_t>(config.getInt("boardHeight"));
        options.cols = static_cast<size_t>(config.getInt("boardWidth"));
        options.spacingMeters = config.getDouble("boardSquareSize");
        options.tagSpacing = config.getDouble("tagSpacingRatio");
        options.imageSize = frameInput.size();

        options.maxIter = static_cast<size_t>(config.getInt("maxIter"));
        options.timeCalibration = config.getBool("timeCalibration");

        calibrator = std::make_unique<Calibrator>(options);
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

        const std::string fmt("{:%Y_%m_%d_%H_%M_%S}");
        const std::string timeString
            = fmt::format(fmt, fmt::localtime(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())));
        const std::string fileName = "camera_imu_calibration_" + cameraID + "-" + timeString + ".xml";

        auto filePath = (outputDir / fileName).string();
        return filePath;
    }

    void saveCalibration(
        const CameraCalibration::CalibrationResult& intrinsicResult,
        const IccCalibrator::CalibrationResult& result) {
        const auto filePath = saveFilePath();

        cv::FileStorage fs(filePath, cv::FileStorage::WRITE);

        if (!fs.isOpened()) {
            log.warning << "Failed to write to calibration file: " << filePath << dv::logEnd;
            return;
        }

        std::vector<double> vec{0.1, 0.9, 0.2, 0.8, 0.3, 0.7, 0.4, 0.6, 0.5, 1};
        cv::Mat m2(1, vec.size(), CV_64FC1, vec.data());

        std::vector<double> projCopy(intrinsicResult.projection);
        cv::Mat projMat(1, intrinsicResult.projection.size(), CV_64FC1, projCopy.data());
        fs << "projection_coefficients" << projMat;
        std::vector<double> distCopy(intrinsicResult.distortion);
        cv::Mat distMat(1, intrinsicResult.distortion.size(), CV_64FC1, distCopy.data());
        fs << "distortion_coefficients" << distMat;
        cv::Mat intrErr;
        cv::eigen2cv(intrinsicResult.err_info.mean, intrErr);
        fs << "intrinsic_calibration_reprojection_error" << intrErr;

        fs << "board_width" << config.getInt("boardWidth");
        fs << "board_height" << config.getInt("boardHeight");
        fs << "calibration_pattern" << config.getString("calibrationPattern");
        fs << "board_square_size" << config.getDouble("boardSquareSize");

        fs << "calibration_converged" << result.converged;
        fs << "time_offset_cam_imu" << result.t_cam_imu;
        cv::Mat trans;
        cv::eigen2cv(result.T_cam_imu, trans);
        fs << "transformation_cam_imu" << trans;

        fs << "mean_reprojection_error" << result.error_info.meanReprojectionError;
        fs << "mean_accelerometer_error" << result.error_info.meanAccelerometerError;
        fs << "mean_gyroscope_error" << result.error_info.meanGyroscopeError;

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
        CameraCalibration::printResult(intrinsicsResult.value(), ss);
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
        IccCalibrator::printResult(result, ss);
        string2dvLog(ss.str());

        // Save the calibration to a text file
        saveCalibration(intrinsicsResult.value(), result);
    }
};

registerModuleClass(ImuCamCalibration)
