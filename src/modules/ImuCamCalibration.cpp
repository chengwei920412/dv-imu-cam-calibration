#include "utilities/Calibrator.hpp"
#include "utilities/CalibratorBase.hpp"

#include <aslam/cameras.hpp>

#include <dv-sdk/module.hpp>

#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/io/mono_camera_writer.hpp>
#include <filesystem>
#include <fmt/chrono.h>
#include <fstream>
#include <regex>
#include <string>
#include <thread>

namespace pt = boost::property_tree;
namespace fs = std::filesystem;

std::string getTimeString() {
    return fmt::format(
        "{:%Y-%m-%dT%H-%M-%SZ}",
        fmt::localtime(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())));
}

namespace ImuCamModelTypes {
const std::string RADTAN = "Pinhole-RadialTangential";
const std::string EQUIDISTANT = "Pinhole-Equidistant";

} // namespace ImuCamModelTypes

namespace QualityStrings {
const std::string EXCELLENT = "excellent";
const std::string GOOD = "good";
const std::string POOR = "poor";
const std::string BAD = "bad";
} // namespace QualityStrings

class ImuCamCalibration : public dv::ModuleBase {
protected:
    // Calibrator options
    CalibratorUtils::Options mOptions;
    std::string mCalibrationModel = ImuCamModelTypes::RADTAN;

    std::thread mThreadCalibrate;
    std::string mQuality;
    std::unique_ptr<CalibratorBase> mCalibrator = nullptr;
    std::unique_ptr<dv::io::MonoCameraWriter> mDataLog = nullptr;
    using DVMessage = std::variant<dv::TimedKeyPointPacket, dv::Frame, dv::IMU>;
    using DVStreamAndMessage = std::pair<std::string, DVMessage>;
    std::multimap<int64_t, DVStreamAndMessage> mDataLogBuffer;
    std::string mTimestampString;
    std::vector<int64_t> mTimes;
    int64_t mStartTime = -1;
    int64_t mWarmUpDuration = 100'000;

    boost::circular_buffer<dv::Frame> mLeftFrames = boost::circular_buffer<dv::Frame>(5);
    boost::circular_buffer<dv::Frame> mRightFrames = boost::circular_buffer<dv::Frame>(5);
    dv::io::MonoCameraWriter::Config mWriterConfig;

    enum CollectionState { BEFORE_COLLECTING, DURING_COLLECTING, AFTER_COLLECTING, CALIBRATING, CALIBRATED };

    CollectionState collectionState = BEFORE_COLLECTING;

    static std::optional<double> estimateFrequency(const std::vector<int64_t>& timestamps) {
        std::vector<double> freq;
        if (timestamps.size() < 2) {
            return std::nullopt;
        }

        for (auto iter = std::next(timestamps.begin()); iter < timestamps.end(); iter++) {
            // We are interested in frequencies in 1Hz to 1000Hz, so converting to milliseconds, this should give a good
            // enough approximation
            int64_t t1 = *std::prev(iter);
            int64_t t2 = *iter;
            freq.push_back(1e+6 / static_cast<double>(t2 - t1));
        }

        return std::accumulate(freq.begin(), freq.end(), 0.) / static_cast<double>(freq.size());
    }

    std::optional<double> imuUpdateRate = std::nullopt;

    void writeDataLogBuffer(const int64_t leaveLastMicroseconds = 0) {
        if (mDataLogBuffer.empty() || mDataLog == nullptr) {
            // Nothing to write
            return;
        }

        // Write the data up to last - minus a bit microseconds
        const auto lastTs = mDataLogBuffer.rbegin()->first;
        int64_t lastWrittenTs = -1;
        for (const auto& [ts, streamAndData] : mDataLogBuffer) {
            if (ts > lastTs - leaveLastMicroseconds) {
                break;
            }
            lastWrittenTs = ts;
            const auto& [stream, data] = streamAndData;
            if (const auto* pval = std::get_if<dv::TimedKeyPointPacket>(&data)) {
                mDataLog->writePacket(*pval, stream);
            } else if (const auto* pval = std::get_if<dv::Frame>(&data)) {
                mDataLog->writePacket(*pval, stream);
            } else if (const auto* pval = std::get_if<dv::IMU>(&data)) {
                mDataLog->writeImu(*pval, stream);
            } else {
                throw std::runtime_error("Unknown packet type");
            }
        }

        // Erase the data that has been written
        if (lastWrittenTs != -1) {
            const auto it = mDataLogBuffer.find(lastWrittenTs);
            mDataLogBuffer.erase(mDataLogBuffer.begin(), it);
        }
    }

public:
    static void initInputs(dv::InputDefinitionList& in) {
        in.addFrameInput("left");
        in.addIMUInput("imu", true);
        in.addFrameInput("right", true);
    }

    static void initOutputs(dv::OutputDefinitionList& out) {
        out.addFrameOutput("left");
        out.addFrameOutput("right");
    }

    static const char* initDescription() {
        return ("Calibrate the IMU with respect to the event camera.");
    }

    static void initConfigOptions(dv::RuntimeConfig& config) {
        // Camera calibration file output
        config.add(
            "outputCalibrationDirectory",
            dv::ConfigOption::directoryOption(
                "Path to a directory to save the calibration settings in",
                dv::portable_get_user_home_directory()));

        // Calibration pattern
        config.add(
            "numPatternColumns",
            dv::ConfigOption::intOption("Number of columns in the calibration pattern", 6, 1, 50));
        config.add(
            "numPatternRows",
            dv::ConfigOption::intOption("Number of rows in the calibration pattern", 6, 1, 50));
        config.add(
            "markerSize",
            dv::ConfigOption::floatOption("Size of a calibration pattern element in meters", 0.05, 0.0, 1.0));
        config.add(
            "markerSpacing",
            dv::ConfigOption::floatOption("Ratio of space between tags to tagSize (AprilGrid only)", 0.3, 0.0, 1.0));
        config.add(
            "patternType",
            dv::ConfigOption::listOption(
                "Type of calibration pattern to use",
                "aprilGrid",
                {"aprilGrid", "asymmetricCirclesGrid", "chessboard"},
                false));

        // All the supported models are pinhole projection camera model. What change is the distortion.
        config.add(
            "calibrationModel",
            dv::ConfigOption::listOption(
                "Calibration model to use",
                ImuCamModelTypes::RADTAN,
                {ImuCamModelTypes::RADTAN, ImuCamModelTypes::EQUIDISTANT},
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
        config.add("calibrationFinished", dv::ConfigOption::boolOption("Calibration finished", false, true));

        // Optimization options
        config.add(
            "maxIter",
            dv::ConfigOption::intOption("Maximum number of iteration of calibration optimization problem", 50, 1, 100));
        config.add(
            "timeCalibration",
            dv::ConfigOption::boolOption("If true, time offset between the sensors will be calibrated", true));

        // IMU noise parameters
        config.add("recordData", dv::ConfigOption::boolOption("Record collected data in the output directory", true));

        config.setPriorityOptions(
            {"outputCalibrationDirectory",
             "calibrationModel",
             "boardHeight",
             "boardWidth",
             "markerSize",
             "patternType",
             "startCollecting",
             "stopCollecting",
             "discard",
             "calibrate"});
    }

    void handleCollectionState() {
        if (mCalibrationModel != config.getString("calibrationModel")) {
            mCalibrationModel = config.getString("calibrationModel");
            initializeCalibrator();
            collectionState = BEFORE_COLLECTING;
            log.info(fmt::format("Calibration model has changed to : {0}", mCalibrationModel));
        }
        if (mCalibrator == nullptr) {
            initializeCalibrator();
            collectionState = BEFORE_COLLECTING;
        }

        // Handle user input
        switch (collectionState) {
            case BEFORE_COLLECTING: {
                if (config.getBool("startCollecting")) {
                    collectionState = DURING_COLLECTING;
                    mCalibrator->startCollecting();
                    log.info("Started collecting images");
                }
                break;
            }
            case DURING_COLLECTING: {
                if (config.getBool("stopCollecting")) {
                    collectionState = AFTER_COLLECTING;
                    mCalibrator->stopCollecting();
                    log.info("Stopped collecting images");
                }
                break;
            }
            case AFTER_COLLECTING: {
                if (config.getBool("calibrate")) {
                    mDataLog = nullptr;

                    collectionState = CALIBRATING;
                    mThreadCalibrate = std::thread([&]() {
                        calibrate(inputs.isConnected("imu"));
                    });
                }

                if (config.getBool("discard")) {
                    collectionState = BEFORE_COLLECTING;
                    if (mThreadCalibrate.joinable()) {
                        mThreadCalibrate.join();
                    }
                    log.info("Discarded all collected data");
                    mCalibrator->reset();
                    initializeCalibrator();
                }
                break;
            }
            case CALIBRATING: {
                break;
            }
            case CALIBRATED: {
                if (config.getBool("discard")) {
                    collectionState = BEFORE_COLLECTING;
                    if (mThreadCalibrate.joinable()) {
                        mThreadCalibrate.join();
                    }
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
            case CALIBRATING: {
                config.setBool("startCollecting", true);
                config.setBool("stopCollecting", true);
                config.setBool("discard", true);
                config.setBool("calibrate", true);
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

    void setupCalibrator() {
        if (mCalibrator != nullptr) {
            mCalibrator->reset();
        }
        // All the supported models are pinhole projection camera model. What change is the distortion.
        if (config.getString("calibrationModel") == ImuCamModelTypes::EQUIDISTANT) {
            mCalibrator = std::make_unique<Calibrator<
                aslam::cameras::EquidistantDistortedPinholeCameraGeometry,
                aslam::cameras::EquidistantDistortion>>(mOptions);
        } else {
            mCalibrator = std::make_unique<
                Calibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>>(
                mOptions);
        }
    }

    void configUpdate() override {
        handleCollectionState();
    }

    void initializeCalibrator() {
        const auto frameInput = inputs.getFrameInput("left");
        mTimestampString = getTimeString();
        mOptions = CalibratorUtils::Options();

        mOptions.pattern = getPatternType();
        mOptions.cols = static_cast<size_t>(config.getInt("numPatternColumns"));
        mOptions.rows = static_cast<size_t>(config.getInt("numPatternRows"));
        mOptions.spacingMeters = static_cast<double>(config.getFloat("markerSize"));
        mOptions.patternSpacing = static_cast<double>(config.getFloat("markerSpacing"));
        mOptions.cameraInitialSettings.emplace_back().imageSize = frameInput.size();

        // Add config for the second camera if it is connected
        const auto rightInput = inputs.getFrameInput("right");
        if (rightInput.isConnected()) {
            mOptions.cameraInitialSettings.emplace_back().imageSize = rightInput.size();
        }

        mOptions.maxIter = static_cast<size_t>(config.getInt("maxIter"));
        mOptions.timeCalibration = config.getBool("timeCalibration");

        if (imuUpdateRate.has_value()) {
            mOptions.imuParameters.updateRate = *imuUpdateRate;
        }

        setupCalibrator();

        // TODO: wrap a class around the MonoCameraWriter and the StereoCameraWriter
        if (config.getBool("recordData")) {
            mWriterConfig.cameraName = getCameraID("left");
            mWriterConfig.addFrameStream(frameInput.size(), "frames");
            if (inputs.getIMUInput("imu").isConnected()) {
                mWriterConfig.addImuStream("imu");
            }
            mWriterConfig.addFrameStream(frameInput.size(), "left_frames", frameInput.getOriginDescription());
            mWriterConfig.addStream<dv::TimedKeyPointPacket>("left_markers");
            if (rightInput.isConnected()) {
                mWriterConfig.addFrameStream(rightInput.size(), "right_frames", rightInput.getOriginDescription());
                mWriterConfig.addStream<dv::TimedKeyPointPacket>("right_markers");
            }
        }
    }

    ImuCamCalibration() : mWriterConfig(getCameraID("left")) {
        // Input output
        const auto frameInput = inputs.getFrameInput("left");
        const auto inputSize = frameInput.size();
        const auto description = frameInput.getOriginDescription();
        outputs.getFrameOutput("left").setup(inputSize.width, inputSize.height, description);

        if (inputs.isConnected("right")) {
            outputs.getFrameOutput("right").setup(inputs.getFrameInput("right"));
        } else {
            // Setup using left camera info, but it will not output anything
            outputs.getFrameOutput("right").setup(inputs.getFrameInput("left"));
        }

        // If imu input is connected, do not initialize and wait until imu frequency is estimated
        if (!inputs.getIMUInput("imu").isConnected()) {
            initializeCalibrator();
        }

        // Unclick all the buttons and update the state
        config.setBool("startCollecting", false);
        config.setBool("stopCollecting", false);
        config.setBool("discard", false);
        config.setBool("calibrate", false);
        handleCollectionState();
    }

    ~ImuCamCalibration() override {
        if (mThreadCalibrate.joinable()) {
            mThreadCalibrate.join();
        }
        mCalibrator->reset();
        writeDataLogBuffer();
    }

    void logObservationData() {
        auto detection = mCalibrator->getLatestObservations();
        if (!detection.first.empty() && !detection.second.empty()) {
            dv::runtime_assert(
                detection.first.size() == detection.second.size(),
                "Observation count does not match image count!");
        } else {
            return;
        }
        // TODO: Log both streams
        size_t index = 0;
        for (size_t index = 0; index < detection.first.size(); index++) {
            int64_t timestamp = detection.first[index].timestamp;
            dv::TimedKeyPointPacket markers;
            std::vector<uint32_t> ids;
            size_t count = detection.second[index]->getCornersIdx(ids);

            markers.elements.reserve(count);
            for (uint32_t id : ids) {
                Eigen::Vector2d cornerEigen;
                detection.second[index]->imagePoint(id, cornerEigen);
                markers.elements.emplace_back(
                    dv::Point2f(static_cast<float>(cornerEigen(0)), static_cast<float>(cornerEigen(1))),
                    1.f,
                    -1.f,
                    1.f,
                    0,
                    id,
                    timestamp);
            }
            if (index == 0) {
                const auto ts = detection.first[index].timestamp;
                mDataLogBuffer.emplace(
                    std::make_pair(ts, std::make_pair("left_frames", dv::Frame(ts, detection.first[index].image))));
                mDataLogBuffer.emplace(std::make_pair(ts, std::make_pair("left_markers", markers)));
            } else {
                const auto ts = detection.first[index].timestamp;
                mDataLogBuffer.emplace(
                    std::make_pair(ts, std::make_pair("right_frames", dv::Frame(ts, detection.first[index].image))));
                mDataLogBuffer.emplace(std::make_pair(ts, std::make_pair("right_markers", markers)));
            }
        }

        // Write the log data from the buffer only if it is more than 1 second old
        writeDataLogBuffer(1000000);
    }

    std::optional<size_t> estimateImuFrequency(const dv::IMUPacket& packet) {
        for (const auto& measurement : packet.elements) {
            mTimes.push_back(measurement.timestamp);
        }
        if (mTimes.size() > 10) {
            auto result = estimateFrequency(mTimes);
            mTimes.clear();
            return result;
        } else {
            return std::nullopt;
        }
    }

    const dv::Frame& closestRightFrame(const int64_t timestamp) {
        auto iter
            = std::min_element(mRightFrames.begin(), mRightFrames.end(), [timestamp](const auto& a, const auto& b) {
                  return std::abs(a.timestamp - timestamp) < std::abs(b.timestamp - timestamp);
              });
        return *iter;
    }

    void run() override {
        if (mCalibrator == nullptr) {
            handleCollectionState();
        }
        // Process IMU input
        if (inputs.isConnected("imu")) {
            auto imuInput = inputs.getIMUInput("imu");
            if (auto imuData = imuInput.data()) {
                if (!mCalibrator) {
                    if (mStartTime < 0) {
                        mStartTime = imuData.front().timestamp;
                    }

                    // Skip some data during warm up, the timestamps are not well aligned during startup
                    if (imuData.front().timestamp - mStartTime < mWarmUpDuration) {
                        return;
                    }

                    if (auto frequency = estimateImuFrequency(*imuData.getBasePointer()); frequency.has_value()) {
                        imuUpdateRate = static_cast<double>(*frequency);
                    }
                    // wait for frequency estimation
                    return;
                }

                for (const auto& singleImu : imuData) {
                    mCalibrator->addImu(
                        singleImu.timestamp,
                        singleImu.getAngularVelocities().cast<double>(),
                        singleImu.getAccelerations().cast<double>());
                }
                if (collectionState == DURING_COLLECTING) {
                    for (const auto& singleImu : imuData) {
                        const auto ts = singleImu.timestamp;
                        mDataLogBuffer.emplace(std::make_pair(ts, std::make_pair("imu", singleImu)));
                    }
                }
            }
        }

        const auto& rightInput = inputs.getFrameInput("right");
        const bool stereo = rightInput.isConnected();

        if (stereo) {
            if (auto frame = rightInput.data()) {
                mRightFrames.push_back(*frame.getBasePointer());
            }
        }

        // Process frame input
        auto frameInput = inputs.getFrameInput("left");
        if (auto frame = frameInput.data()) {
            mLeftFrames.push_back(*frame.getBasePointer());
        }

        if (stereo && !mRightFrames.full()) {
            return;
        }

        if (mLeftFrames.full()) {
            std::vector<CalibratorUtils::StampedImage> images;
            dv::Frame& left = mLeftFrames.front();
            if (left.image.channels() == 3) {
                cv::Mat gray;
                cv::cvtColor(left.image, gray, cv::COLOR_BGR2GRAY);
                images.emplace_back(gray, left.timestamp);
            } else {
                images.emplace_back(left.image, left.timestamp);
            }

            if (stereo) {
                const dv::Frame& right = closestRightFrame(left.timestamp);
                // Pass in left timestamp so it can be easily pair-matched
                if (right.image.channels() == 3) {
                    cv::Mat gray;
                    cv::cvtColor(right.image, gray, cv::COLOR_BGR2GRAY);
                    images.emplace_back(gray, right.timestamp);
                } else {
                    images.emplace_back(right.image, left.timestamp);
                }
            }
            mCalibrator->addImages(images);

            // Output preview image
            auto previews = mCalibrator->getPreviewImages();
            if (!previews.empty()) {
                if (collectionState == BEFORE_COLLECTING) {
                    if (!inputs.isConnected("imu")) {
                        cv::putText(
                            previews[0].image,
                            "No IMU data",
                            cv::Point(20, previews[0].image.rows - 20),
                            cv::FONT_HERSHEY_DUPLEX,
                            1.0,
                            cv::Scalar(0, 165, 255),
                            2);
                    }
                    cv::putText(
                        previews[0].image,
                        config.getString("calibrationModel"),
                        cv::Point(20, 80),
                        cv::FONT_HERSHEY_DUPLEX,
                        .5,
                        cv::Scalar(255, 0, 0),
                        2);
                } else if (collectionState == CALIBRATED) {
                    drawQuality(previews[0].image);
                }
                outputs.getFrameOutput("left") << previews[0].timestamp << previews[0].image << dv::commit;
                if (previews.size() == 2) {
                    outputs.getFrameOutput("right") << previews[1].timestamp << previews[1].image << dv::commit;
                }
            }
            if (mDataLog) {
                logObservationData();
            }
            mLeftFrames.pop_front();
        }
    }

protected:
    fs::path getCalibrationSaveDirectory() {
        const auto frameInput = inputs.getFrameInput("left");
        const auto cameraID = frameInput.getOriginDescription();
        fs::path outputDir{config.getString("outputCalibrationDirectory")};

        if (outputDir.empty()) {
            outputDir = dv::portable_get_user_home_directory();
        }

        const std::string dirName = "dv_calibration_" + mTimestampString;
        auto outputDirWithTs = outputDir / dirName;

        if (!fs::is_directory(outputDirWithTs)) {
            fs::create_directories(outputDirWithTs);
        }

        return outputDirWithTs;
    }

    CalibratorUtils::PatternType getPatternType() {
        const auto pattern = config.getString("patternType");
        if (pattern == "aprilGrid") {
            return CalibratorUtils::PatternType::APRIL_GRID;
        } else if (pattern == "asymmetricCirclesGrid") {
            return CalibratorUtils::PatternType::ASYMMETRIC_CIRCLES_GRID;
        } else if (pattern == "chessboard") {
            return CalibratorUtils::PatternType::CHESSBOARD;
        } else {
            throw std::runtime_error("Unknown calibration pattern: " + pattern);
        }
    }

    int getPatternColumns() {
        return config.getInt("numPatternColumns");
    }

    int getPatternRows() {
        return config.getInt("numPatternRows");
    }

    int getInternalPatternColumns() {
        int cols = getPatternColumns();
        if (getPatternType() == CalibratorUtils::PatternType::CHESSBOARD) {
            // In case of chessboard we actually detect the inner corners
            cols -= 1;
        }
        return cols;
    }

    int getInternalPatternRows() {
        int rows = getPatternRows();
        if (getPatternType() == CalibratorUtils::PatternType::CHESSBOARD) {
            // In case of chessboard we actually detect the inner corners
            rows -= 1;
        }
        return rows;
    }
    std::string getCameraID(const std::string& inputName) {
        const auto frameInput = inputs.getFrameInput(inputName);
        const auto originDescription = frameInput.getOriginDescription();
        static const std::regex filenameCleanupRegex{"[^a-zA-Z-_\\d]"};
        auto cameraID = std::regex_replace(originDescription, filenameCleanupRegex, "_");
        if ((cameraID[0] == '-') || (std::isdigit(cameraID[0]) != 0)) {
            cameraID = "_" + cameraID;
        }
        return cameraID;
    }

    bool isInputMaster(const std::string& inputName) const {
        for (const auto& child :
             inputs.getFrameInput(inputName).infoNode().getParent().getParent().getParent().getChildren()) {
            if (child.getName() == "sourceInfo") {
                return child.getAttribute<dv::Config::AttributeType::BOOL>("deviceIsMaster").value;
            }
        }
        return false;
    }

    cv::Size getInputResolution(const std::string& inputName) {
        const auto frameInput = inputs.getFrameInput(inputName);
        return {frameInput.sizeX(), frameInput.sizeY()};
    }

    static std::vector<float> getIdentityMatrixVector() {
        return std::vector<float>{1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
    }

    dv::camera::calibrations::CameraCalibration::Metadata
        getCameraCalibrationMetadata(const CameraCalibrationUtils::CalibrationResult& intrinsicResult) {
        std::stringstream ssError;
        ssError << "Mean: " << intrinsicResult.err_info.mean.transpose()
                << " Standard deviation: " << intrinsicResult.err_info.std.transpose();

        auto meanStd = (intrinsicResult.err_info.std.x() + intrinsicResult.err_info.std.y()) / 2.0;
        if (meanStd < 0.1) {
            mQuality = QualityStrings::EXCELLENT;
        } else if (meanStd < 0.2) {
            mQuality = QualityStrings::GOOD;
        } else if (meanStd < 0.5) {
            mQuality = QualityStrings::POOR;
        } else {
            mQuality = QualityStrings::BAD;
        }

        return {
            cv::Size(getPatternColumns(), getPatternRows()),
            cv::Size(getInternalPatternColumns(), getInternalPatternRows()),
            config.getString("patternType"),
            config.getFloat("markerSize"),
            config.getFloat("markerSpacing"),
            meanStd,
            mTimestampString,
            mQuality,
            "",
            std::nullopt};
    }

    dv::camera::calibrations::CameraCalibration getIntrinsicCalibrationData(
        const CameraCalibrationUtils::CalibrationResult& res,
        const std::string& position,
        const std::string& inputName) {
        const Eigen::Matrix<float, 4, 4, Eigen::RowMajor> floatTransform = res.baseline.cast<float>().eval();

        dv::camera::DistortionModel distortionModel = dv::camera::DistortionModel::RadTan;
        if (config.getString("calibrationModel") == ImuCamModelTypes::EQUIDISTANT) {
            distortionModel = dv::camera::DistortionModel::Equidistant;
        }

        dv::camera::calibrations::CameraCalibration cal(
            getCameraID(inputName),
            position,
            isInputMaster(inputName),
            getInputResolution(inputName),
            cv::Point2f(static_cast<float>(res.projection.at(2)), static_cast<float>(res.projection.at(3))),
            cv::Point2f(static_cast<float>(res.projection.at(0)), static_cast<float>(res.projection.at(1))),
            std::vector<float>(res.distortion.begin(), res.distortion.end()),
            distortionModel,
            std::vector<float>(floatTransform.data(), floatTransform.data() + 16),
            getCameraCalibrationMetadata(res));

        return cal;
    }

    std::tuple<float, float, cv::Point3f, cv::Point3f, float, float, float, float, float, float>
        getIMUCharacteristics() {
        float omega_max, acc_max, omega_offset_var, acc_offset_var, omega_noise_density, acc_noise_density,
            omega_noise_random_walk, acc_noise_random_walk;

        cv::Point3f omega_offset_avg, acc_offset_avg;

        const auto cameraId = getCameraID("left");
        if (cameraId.substr(0, 9) == "DVXplorer") {
            omega_max = 34.89f;
            acc_max = 156.96f;
            omega_offset_avg = {0, 0, 0};
            acc_offset_avg = {0, 0, 0};
            omega_offset_var = 0.03f;
            acc_offset_var = 0.1f;
            omega_noise_density = 8.0e-5f;
            acc_noise_density = 1.0e-3f;
            omega_noise_random_walk = 4.0e-6f;
            acc_noise_random_walk = 4.0e-5f;
        } else if (cameraId.substr(0, 5) == "DAVIS") {
            omega_max = 7.8f;
            acc_max = 176.0f;
            omega_offset_avg = {0, 0, 0};
            acc_offset_avg = {0, 0, 0};
            omega_offset_var = 0.03f;
            acc_offset_var = 0.1f;
            omega_noise_density = 0.00018f;
            acc_noise_density = 0.002f;
            omega_noise_random_walk = 0.001f;
            acc_noise_random_walk = 4.0e-5f;
        } else {
            log.warning << "Could not determine the camera type. IMU noise characteristics are unknown." << dv::logEnd;
        }

        return std::make_tuple(
            omega_max,
            acc_max,
            omega_offset_avg,
            acc_offset_avg,
            omega_offset_var,
            acc_offset_var,
            omega_noise_density,
            acc_noise_density,
            omega_noise_random_walk,
            acc_noise_random_walk);
    }

    dv::camera::calibrations::IMUCalibration
        getIMUCalibrationData(const IccCalibratorUtils::CalibrationResult& result) {
        const auto& [om, am, ooavg, aoavg, oovar, aovar, onden, anden, onrw, anrw] = getIMUCharacteristics();

        std::vector<float> transData;
        transData.reserve(16);
        for (long row = 0; row < result.T_cam_imu.rows(); ++row) {
            for (long col = 0; col < result.T_cam_imu.cols(); ++col) {
                transData.push_back(static_cast<float>(result.T_cam_imu(row, col)));
            }
        }

        std::stringstream ssCom;
        ssCom << "Time offset usage: t_correct = t_imu - offset"
              << " Mean reprojection error: " << result.error_info.meanReprojectionError
              << " Mean acc error: " << result.error_info.meanAccelerometerError
              << " Mean gyroscope error: " << result.error_info.meanGyroscopeError;

        auto cal = dv::camera::calibrations::IMUCalibration(
            getCameraID("left"),
            om,
            am,
            ooavg,
            aoavg,
            oovar,
            aovar,
            onden,
            anden,
            onrw,
            anrw,
            static_cast<int64_t>(result.t_cam_imu * 1e+6),
            transData,
            dv::camera::calibrations::IMUCalibration::Metadata{mTimestampString, ssCom.str()});

        return cal;
    }

    void drawQuality(cv::Mat& image) {
        cv::Scalar color;
        if (mQuality == QualityStrings::EXCELLENT) {
            color = cv::Scalar(0, 255, 0);
        } else if (mQuality == QualityStrings::GOOD) {
            color = cv::Scalar(145, 255, 0);
        } else if (mQuality == QualityStrings::POOR) {
            color = cv::Scalar(145, 0, 255);
        } else {
            color = cv::Scalar(0, 0, 255);
        }
        cv::putText(
            image,
            fmt::format("Quality: {0}", mQuality),
            cv::Point(20, image.rows - 20),
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            color,
            2);
    }

    void saveIntrinsicCalibration(const std::vector<CameraCalibrationUtils::CalibrationResult>& intrinsicResult) {
        const auto saveDir = getCalibrationSaveDirectory();

        const auto filePath = saveDir / "calibration.json";

        dv::camera::CalibrationSet calib;

        calib.addCameraCalibration(getIntrinsicCalibrationData(intrinsicResult[0], "left", "left"));
        log.info << "Calibration quality left camera: " << calib.getCameraCalibration("left")->metadata->quality
                 << dv::logEnd;
        if (intrinsicResult.size() > 1) {
            calib.addCameraCalibration(getIntrinsicCalibrationData(intrinsicResult[1], "right", "right"));
            log.info << "Calibration quality right camera: " << calib.getCameraCalibration("right")->metadata->quality
                     << dv::logEnd;
        }

        calib.writeToFile(filePath.string());

        log.info << "Saved intrinsic calibration to a file: " << filePath << dv::logEnd;
    }

    void saveCalibration(
        const std::vector<CameraCalibrationUtils::CalibrationResult>& intrinsicResult,
        const IccCalibratorUtils::CalibrationResult& result) {
        log.info << "Saving calibration..." << dv::logEnd;
        const auto saveDir = getCalibrationSaveDirectory();

        const auto filePath = saveDir / "calibration.json";

        dv::camera::CalibrationSet calib;

        calib.addCameraCalibration(getIntrinsicCalibrationData(intrinsicResult[0], "left", "left"));
        log.info << "Calibration quality left camera: " << calib.getCameraCalibration("left")->metadata->quality
                 << dv::logEnd;
        if (intrinsicResult.size() > 1) {
            calib.addCameraCalibration(getIntrinsicCalibrationData(intrinsicResult[1], "right", "right"));
            log.info << "Calibration quality right camera: " << calib.getCameraCalibration("right")->metadata->quality
                     << dv::logEnd;
        }
        calib.addImuCalibration(getIMUCalibrationData(result));

        calib.writeToFile(filePath.string());

        log.info << "Saved calibration to a file: " << filePath << dv::logEnd;
    }

    void calibrate(const bool calibrateImu) {
        if (config.getBool("recordData")) {
            mDataLog = std::make_unique<dv::io::MonoCameraWriter>(
                fs::path(getCalibrationSaveDirectory()) / "data.aedat4",
                mWriterConfig);
        }

        std::ofstream outLog(getCalibrationSaveDirectory() / "log.txt");
        outLog << "Calibrating begins..." << std::endl;

        mCalibrator->print(outLog);

        const auto& rdbuf = std::cout.rdbuf();
        std::cout.rdbuf(outLog.rdbuf());

        log.info("Calibrating the intrinsics of the camera...");
        auto intrinsicsResult = mCalibrator->calibrateCameraIntrinsics();
        if (!intrinsicsResult.has_value()) {
            throw dv::exceptions::RuntimeError(
                "Failed to calibrate intrinsics! Please check that the pattern was well detected on the images");
        }

        if (calibrateImu) {
            outLog << "Building the problem..." << std::endl;
            mCalibrator->buildProblem();

            // Print the info before optimization
            mCalibrator->getDvInfoBeforeOptimization(outLog);

            // Run the optimization problem
            outLog << "Optimizing..." << std::endl;
            try {
                IccCalibratorUtils::CalibrationResult result = mCalibrator->calibrate();
                // Print the info after optimization
                mCalibrator->getDvInfoAfterOptimization(outLog);

                // Print the result
                outLog << "RESULT" << std::endl;
                IccCalibratorUtils::printResult(result, outLog);

                // Save the calibration to a text file
                saveCalibration(intrinsicsResult.value(), result);
                collectionState = CALIBRATED;
            } catch (std::exception& ex) {
                outLog << ex.what() << std::endl;
                log.error << "Optimization failed. Please make sure that the pattern is detected on all frames in your "
                             "dataset and repeat the calibration"
                          << dv::logEnd;
                initializeCalibrator();
                collectionState = BEFORE_COLLECTING;
            }
        } else {
            saveIntrinsicCalibration(intrinsicsResult.value());
            config.setBool("calibrationFinished", true);
            collectionState = CALIBRATED;
        }
        std::cout.rdbuf(rdbuf);
    }
};

registerModuleClass(ImuCamCalibration)
