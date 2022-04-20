#include "utilities/Calibrator.hpp"

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

namespace fs = boost::filesystem;

int64_t str2int(const std::string& str) {
    char* pEnd;
    return std::strtoll(str.c_str(), &pEnd, 10);
}

template<typename GeometryType, typename DistortionType>
void addImagesToCalibrator(
    Calibrator<GeometryType, DistortionType>& calibrator,
    const std::vector<fs::path>& imgLeftPaths,
    const std::vector<fs::path>& imgRightPaths = {}) {
    if (imgRightPaths.size() != 0) {
        EXPECT_TRUE(imgLeftPaths.size() != imgRightPaths.size());
        return;
    }
    for (size_t i = 0; i < imgLeftPaths.size(); i++) {
        CalibratorUtils::StampedImage imageLeft;
        cv::Mat imgLeft = cv::imread(imgLeftPaths[i].string(), cv::IMREAD_GRAYSCALE);
        int64_t tsLeft = str2int(imgLeftPaths[i].stem().string());
        if (imgLeft.channels() == 3) {
            cv::Mat gray;
            cv::cvtColor(imgLeft, gray, cv::COLOR_BGR2GRAY);
            imageLeft = CalibratorUtils::StampedImage(gray, tsLeft);
        } else {
            imageLeft = CalibratorUtils::StampedImage(imgLeft, tsLeft);
        }
        if (imgRightPaths.size() > 0) {
            CalibratorUtils::StampedImage imageRight;
            cv::Mat imgRight = cv::imread(imgRightPaths[i].string(), cv::IMREAD_GRAYSCALE);
            int64_t tsRight = str2int(imgRightPaths[i].stem().string());
            if (imgRight.channels() == 3) {
                cv::Mat gray;
                cv::cvtColor(imgRight, gray, cv::COLOR_BGR2GRAY);
                imageRight = CalibratorUtils::StampedImage(gray, tsLeft);
            } else {
                imageRight = CalibratorUtils::StampedImage(imgRight, tsLeft);
            }
            calibrator.addImages({imageLeft, imageRight});
        } else {
            calibrator.addImages({imageLeft});
        }
    }
}

template<typename GeometryType, typename DistortionType>
void addImuToCalibrator(Calibrator<GeometryType, DistortionType>& calibrator, const std::vector<fs::path>& imuPaths) {
    for (const auto& path : imuPaths) {
        std::ifstream infile(path.string());
        std::string line;

        std::getline(infile, line);
        int64_t ts = str2int(line);

        std::getline(infile, line);
        double gX = std::stod(line) * M_PI / 180.0;

        std::getline(infile, line);
        double gY = std::stod(line) * M_PI / 180.0;

        std::getline(infile, line);
        double gZ = std::stod(line) * M_PI / 180.0;

        std::getline(infile, line);
        double aX = std::stod(line) * 9.81;

        std::getline(infile, line);
        double aY = std::stod(line) * 9.81;

        std::getline(infile, line);
        double aZ = std::stod(line) * 9.81;

        Eigen::Vector3d gyro(gX, gY, gZ);
        Eigen::Vector3d acc(aX, aY, aZ);
        calibrator.addImu(ts, gyro, acc);
    }
}

/**
 * Images Calibration only
 */

TEST(CalibratorNoImuTest, EquidistantDistortionModel) {
    ////
    /// Prepare the calibrator
    ////
    CalibratorUtils::Options options;
    options.cols = 6;
    options.rows = 6;
    options.spacingMeters = 0.088;
    options.patternSpacing = 0.3;
    options.pattern = CalibratorUtils::PatternType::APRIL_GRID;
    options.cameraInitialSettings.emplace_back().imageSize = cv::Size(1280, 1024);
    options.maxIter = 50;
    options.timeCalibration = false;

    Calibrator<aslam::cameras::EquidistantDistortedPinholeCameraGeometry, aslam::cameras::EquidistantDistortion>
        calibrator(options);
    calibrator.startCollecting();

    ////
    /// Useful paths
    ////
    fs::path thisPath(__FILE__);
    fs::path imgsPath = thisPath.parent_path() / "test_files" / "GOPRO" / "img";

    ////
    /// Add images
    ////

    std::vector<fs::path> imgPaths;
    for (fs::directory_iterator itr(imgsPath); itr != fs::directory_iterator(); ++itr) {
        imgPaths.push_back(itr->path());
    }
    std::sort(imgPaths.begin(), imgPaths.end());

    // We don't need many images for testing
    bool useAll = true;
    if (!useAll) {
        const size_t startIdx = 1000;
        const size_t nIdx = 25;
        assert(imgPaths.size() > startIdx + nIdx);
        imgPaths = std::vector<fs::path>(imgPaths.begin() + startIdx, imgPaths.begin() + startIdx + nIdx);
    }

    std::cout << "Testing using " << imgPaths.size() << " images" << std::endl;

    addImagesToCalibrator<
        aslam::cameras::EquidistantDistortedPinholeCameraGeometry,
        aslam::cameras::EquidistantDistortion>(calibrator, imgPaths);

    ////
    /// Try to calibrate - this is the actual test
    ////

    calibrator.stopCollecting();

    calibrator.print(std::cout);

    auto intrResult = calibrator.calibrateCameraIntrinsics();

    EXPECT_TRUE(intrResult.has_value());
    if (!intrResult.has_value()) {
        return;
    }
    const auto res = intrResult.value();

    CameraCalibrationUtils::printResult(res[0], std::cout);

    const auto cameraCalib = res[0];
    const auto distortion = cameraCalib.distortion;

    EXPECT_NEAR(distortion[0], -0.01857137341235418, 0.01);
    EXPECT_NEAR(distortion[1], 0.00050346380036182337, 0.01);
    EXPECT_NEAR(distortion[2], 0.019601472566567236, 0.01);
    EXPECT_NEAR(distortion[3], -0.014804983783675328, 0.01);
}

TEST(CalibratorNoImuTest, RadTanDistortionModel) {
    ////
    /// Prepare the calibrator
    ////
    CalibratorUtils::Options options;
    options.cols = 6;
    options.rows = 6;
    options.spacingMeters = 0.088;
    options.patternSpacing = 0.3;
    options.pattern = CalibratorUtils::PatternType::APRIL_GRID;
    options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
    options.maxIter = 50;
    options.timeCalibration = false;

    Calibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion> calibrator(
        options);
    calibrator.startCollecting();

    ////
    /// Useful paths
    ////

    fs::path thisPath(__FILE__);
    fs::path imgsPath = thisPath.parent_path() / "test_files" / "stereo" / "img_left";

    ////
    /// Add images
    ////

    std::vector<fs::path> imgPaths;
    for (fs::directory_iterator itr(imgsPath); itr != fs::directory_iterator(); ++itr) {
        imgPaths.push_back(itr->path());
    }
    std::sort(imgPaths.begin(), imgPaths.end());

    // We don't need many images for testing
    bool useAll = true;
    if (!useAll) {
        const size_t startIdx = 1000;
        const size_t nIdx = 25;
        assert(imgPaths.size() > startIdx + nIdx);
        imgPaths = std::vector<fs::path>(imgPaths.begin() + startIdx, imgPaths.begin() + startIdx + nIdx);
    }

    std::cout << "Testing using " << imgPaths.size() << " images" << std::endl;

    addImagesToCalibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>(
        calibrator,
        imgPaths);

    ////
    /// Try to calibrate - this is the actual test
    ////

    calibrator.stopCollecting();

    calibrator.print(std::cout);

    auto intrResult = calibrator.calibrateCameraIntrinsics();

    EXPECT_TRUE(intrResult.has_value());
    if (!intrResult.has_value()) {
        return;
    }
    const auto res = intrResult.value();

    CameraCalibrationUtils::printResult(res[0], std::cout);

    const auto cameraCalib = res[0];
    const auto distortion = cameraCalib.distortion;

    EXPECT_NEAR(distortion[0], -0.037492185575901504, 0.001);
    EXPECT_NEAR(distortion[1], 0.0014403509264007896, 0.001);
    EXPECT_NEAR(distortion[2], 0.00041809700179432394, 0.001);
    EXPECT_NEAR(distortion[3], -0.0001080197069080138, 0.001);
}

/**
 * Imu-Camera Calibration, single camera
 */

TEST(CalibratorTest, EquidistantDistortionModel) {
    ////
    /// Prepare the calibrator
    ////
    CalibratorUtils::Options options;
    options.cols = 6;
    options.rows = 6;
    options.spacingMeters = 0.088;
    options.patternSpacing = 0.3;
    options.pattern = CalibratorUtils::PatternType::APRIL_GRID;
    options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
    options.maxIter = 50;
    options.timeCalibration = false;

    Calibrator<aslam::cameras::EquidistantDistortedPinholeCameraGeometry, aslam::cameras::EquidistantDistortion>
        calibrator(options);
    calibrator.startCollecting();

    ////
    /// Useful paths
    ////

    fs::path thisPath(__FILE__);
    fs::path imgsPath = thisPath.parent_path() / "test_files" / "event_mono" / "img";
    fs::path imusPath = thisPath.parent_path() / "test_files" / "event_mono" / "imu";

    ////
    /// Add images
    ////

    std::vector<fs::path> imgPaths;
    for (fs::directory_iterator itr(imgsPath); itr != fs::directory_iterator(); ++itr) {
        imgPaths.push_back(itr->path());
    }
    std::sort(imgPaths.begin(), imgPaths.end());

    // We don't need many images for testing
    bool useAll = true;
    if (!useAll) {
        const size_t startIdx = 1000;
        const size_t nIdx = 25;
        assert(imgPaths.size() > startIdx + nIdx);
        imgPaths = std::vector<fs::path>(imgPaths.begin() + startIdx, imgPaths.begin() + startIdx + nIdx);
    }

    std::cout << "Testing using " << imgPaths.size() << " images" << std::endl;

    addImagesToCalibrator<
        aslam::cameras::EquidistantDistortedPinholeCameraGeometry,
        aslam::cameras::EquidistantDistortion>(calibrator, imgPaths);

    ////
    /// Add IMU measurements
    ////

    std::vector<fs::path> imuPaths;
    for (fs::directory_iterator itr(imusPath); itr != fs::directory_iterator(); ++itr) {
        imuPaths.push_back(itr->path());
    }
    std::sort(imuPaths.begin(), imuPaths.end());

    std::cout << "Testing using " << imuPaths.size() << " IMU measurements" << std::endl;

    addImuToCalibrator<
        aslam::cameras::EquidistantDistortedPinholeCameraGeometry,
        aslam::cameras::EquidistantDistortion>(calibrator, imuPaths);

    ////
    /// Try to calibrate - this is the actual test
    ////

    calibrator.stopCollecting();

    calibrator.print(std::cout);

    auto intrResult = calibrator.calibrateCameraIntrinsics();

    EXPECT_TRUE(intrResult.has_value());
    if (!intrResult.has_value()) {
        return;
    }
    const auto res = intrResult.value();

    CameraCalibrationUtils::printResult(res[0], std::cout);

    const auto cameraCalib = res[0];
    const auto distortion = cameraCalib.distortion;

    EXPECT_NEAR(distortion[0], -0.01857137341235418, 0.01);
    EXPECT_NEAR(distortion[1], 0.012092548557359837, 0.001);
    EXPECT_NEAR(distortion[2], 0.0025734782201408792, 0.001);
    EXPECT_NEAR(distortion[3], -0.0052093652806526778, 0.001);

    calibrator.buildProblem();

    // Print the info before optimization
    calibrator.getDvInfoBeforeOptimization(std::cout);

    // Run the optimization problem
    try {
        IccCalibratorUtils::CalibrationResult result = calibrator.calibrate();
        // Print the info after optimization
        calibrator.getDvInfoAfterOptimization(std::cout);

        // Print the result
        IccCalibratorUtils::printResult(result, std::cout);

        EXPECT_TRUE(result.converged);
    } catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
        std::cout << "Optimization failed. Please make sure that the pattern is detected on all frames in your "
                     "dataset and repeat the calibration"
                  << std::endl;
    }
}

TEST(CalibratorTest, RadTanDistortionModel) {
    ////
    /// Prepare the calibrator
    ////
    CalibratorUtils::Options options;
    options.cols = 6;
    options.rows = 6;
    options.spacingMeters = 0.088;
    options.patternSpacing = 0.3;
    options.pattern = CalibratorUtils::PatternType::APRIL_GRID;
    options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
    options.maxIter = 50;
    options.timeCalibration = false;

    Calibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion> calibrator(
        options);
    calibrator.startCollecting();

    ////
    /// Useful paths
    ////

    fs::path thisPath(__FILE__);
    fs::path imgsPath = thisPath.parent_path() / "test_files" / "stereo" / "img_left";
    fs::path imusPath = thisPath.parent_path() / "test_files" / "stereo" / "imu";

    ////
    /// Add images
    ////

    std::vector<fs::path> imgPaths;
    for (fs::directory_iterator itr(imgsPath); itr != fs::directory_iterator(); ++itr) {
        imgPaths.push_back(itr->path());
    }
    std::sort(imgPaths.begin(), imgPaths.end());

    // We don't need many images for testing
    bool useAll = true;
    if (!useAll) {
        const size_t startIdx = 1000;
        const size_t nIdx = 25;
        assert(imgPaths.size() > startIdx + nIdx);
        imgPaths = std::vector<fs::path>(imgPaths.begin() + startIdx, imgPaths.begin() + startIdx + nIdx);
    }

    std::cout << "Testing using " << imgPaths.size() << " images" << std::endl;

    addImagesToCalibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>(
        calibrator,
        imgPaths);

    ////
    /// Add IMU measurements
    ////

    std::vector<fs::path> imuPaths;
    for (fs::directory_iterator itr(imusPath); itr != fs::directory_iterator(); ++itr) {
        imuPaths.push_back(itr->path());
    }
    std::sort(imuPaths.begin(), imuPaths.end());

    std::cout << "Testing using " << imuPaths.size() << " IMU measurements" << std::endl;

    addImuToCalibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>(
        calibrator,
        imuPaths);

    ////
    /// Try to calibrate - this is the actual test
    ////

    calibrator.stopCollecting();

    calibrator.print(std::cout);

    auto intrResult = calibrator.calibrateCameraIntrinsics();

    EXPECT_TRUE(intrResult.has_value());
    if (!intrResult.has_value()) {
        return;
    }
    const auto res = intrResult.value();

    CameraCalibrationUtils::printResult(res[0], std::cout);

    const auto cameraCalib = res[0];
    const auto distortion = cameraCalib.distortion;

    EXPECT_NEAR(distortion[0], -0.32509976545609448, 0.001);
    EXPECT_NEAR(distortion[1], 0.096499640038551257, 0.001);
    EXPECT_NEAR(distortion[2], 0.00041809700179432394, 0.001);
    EXPECT_NEAR(distortion[3], -0.0001080197069080138, 0.001);

    calibrator.buildProblem();

    // Print the info before optimization
    calibrator.getDvInfoBeforeOptimization(std::cout);

    // Run the optimization problem
    try {
        IccCalibratorUtils::CalibrationResult result = calibrator.calibrate();
        // Print the info after optimization
        calibrator.getDvInfoAfterOptimization(std::cout);

        // Print the result
        IccCalibratorUtils::printResult(result, std::cout);

        EXPECT_TRUE(result.converged);
    } catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
        std::cout << "Optimization failed. Please make sure that the pattern is detected on all frames in your "
                     "dataset and repeat the calibration"
                  << std::endl;
    }
}

/**
 * Stereo Imu-Camera Calibration
 */
TEST(StereoCalibratorTest, StereoEquidistantDistortionModel) {
    ////
    /// Prepare the calibrator
    ////
    CalibratorUtils::Options options;
    options.cols = 6;
    options.rows = 6;
    options.spacingMeters = 0.031;
    options.patternSpacing = 0.29;
    options.pattern = CalibratorUtils::PatternType::APRIL_GRID;
    options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
    options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
    options.maxIter = 50;
    options.timeCalibration = false;
    options.imuParameters.updateRate = 200.0;

    Calibrator<aslam::cameras::EquidistantDistortedPinholeCameraGeometry, aslam::cameras::EquidistantDistortion>
        calibrator(options);

    calibrator.startCollecting();

    ////
    /// Useful paths
    ////

    fs::path thisPath(__FILE__);
    fs::path imgsLeftPath = thisPath.parent_path() / "test_files" / "stereo" / "img_left";
    fs::path imgsRightPath = thisPath.parent_path() / "test_files" / "stereo" / "img_right";
    fs::path imusPath = thisPath.parent_path() / "test_files" / "stereo" / "imu";

    ////
    /// Add images
    ////

    std::vector<fs::path> imgLeftPaths, imgRightPaths;
    for (fs::directory_iterator itr(imgsLeftPath); itr != fs::directory_iterator(); ++itr) {
        imgLeftPaths.push_back(itr->path());
    }
    std::sort(imgLeftPaths.begin(), imgLeftPaths.end());

    for (fs::directory_iterator itr(imgsRightPath); itr != fs::directory_iterator(); ++itr) {
        imgRightPaths.push_back(itr->path());
    }
    std::sort(imgRightPaths.begin(), imgRightPaths.end());

    std::cout << "Testing using " << imgLeftPaths.size() << " images" << std::endl;

    addImagesToCalibrator<
        aslam::cameras::EquidistantDistortedPinholeCameraGeometry,
        aslam::cameras::EquidistantDistortion>(calibrator, imgLeftPaths, imgRightPaths);

    ////
    /// Add IMU measurements
    ////

    std::vector<fs::path> imuPaths;
    for (fs::directory_iterator itr(imusPath); itr != fs::directory_iterator(); ++itr) {
        imuPaths.push_back(itr->path());
    }
    std::sort(imuPaths.begin(), imuPaths.end());

    std::cout << "Testing using " << imuPaths.size() << " IMU measurements" << std::endl;

    addImuToCalibrator<
        aslam::cameras::EquidistantDistortedPinholeCameraGeometry,
        aslam::cameras::EquidistantDistortion>(calibrator, imuPaths);

    ////
    /// Try to calibrate - this is the actual test
    ////

    calibrator.stopCollecting();

    calibrator.print(std::cout);

    auto intrResult = calibrator.calibrateCameraIntrinsics();

    EXPECT_TRUE(intrResult.has_value());
    if (!intrResult.has_value()) {
        return;
    }
    const auto res = intrResult.value();

    CameraCalibrationUtils::printResult(res[0], std::cout);
    CameraCalibrationUtils::printResult(res[1], std::cout);

    const auto cameraCalib0 = res[0];
    const auto distortion0 = cameraCalib0.distortion;

    EXPECT_NEAR(distortion0[0], -0.026196177439606966, 0.01);
    EXPECT_NEAR(distortion0[1], 0.011470443712235325, 0.01);
    EXPECT_NEAR(distortion0[2], -0.031772586000478914, 0.01);
    EXPECT_NEAR(distortion0[3], 0.021363785963385253, 0.001);

    const auto cameraCalib1 = res[1];
    const auto distortion1 = cameraCalib1.distortion;

    EXPECT_NEAR(distortion1[0], -0.014088983574702143, 0.001);
    EXPECT_NEAR(distortion1[1], -0.011596027262493244, 0.001);
    EXPECT_NEAR(distortion1[2], 0.0080855729069350436, 0.001);
    EXPECT_NEAR(distortion1[3], -0.0036355859914529395, 0.001);

    std::cout << "Baseline: " << cameraCalib0.baseline << std::endl;
    //    EXPECT_LT(cameraCalib0.baseline)

    // CAM-IMU calibration

    calibrator.buildProblem();

    calibrator.getDvInfoBeforeOptimization(std::cout);

    try {
        const auto result = calibrator.calibrate();
        calibrator.getDvInfoAfterOptimization(std::cout);

        IccCalibratorUtils::printResult(result, std::cout);
        EXPECT_TRUE(result.converged);
    } catch (std::exception& ex) {
        EXPECT_TRUE(false);
        std::cout << ex.what() << std::endl;
        std::cout << "Optimization failed. Please make sure that the pattern is detected on all frames in your "
                     "dataset and repeat the calibration"
                  << std::endl;
    }
}

TEST(StereoCalibratorTest, StereoRadTanDistortionModel) {
    ////
    /// Prepare the calibrator
    ////
    CalibratorUtils::Options options;
    options.cols = 6;
    options.rows = 6;
    options.spacingMeters = 0.031;
    options.patternSpacing = 0.29;
    options.pattern = CalibratorUtils::PatternType::APRIL_GRID;
    options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
    options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
    options.maxIter = 50;
    options.timeCalibration = false;
    options.imuParameters.updateRate = 200.0;

    Calibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion> calibrator(
        options);

    calibrator.startCollecting();

    ////
    /// Useful paths
    ////

    fs::path thisPath(__FILE__);
    fs::path imgsLeftPath = thisPath.parent_path() / "test_files" / "stereo" / "img_left";
    fs::path imgsRightPath = thisPath.parent_path() / "test_files" / "stereo" / "img_right";
    fs::path imusPath = thisPath.parent_path() / "test_files" / "stereo" / "imu";

    ////
    /// Add images
    ////

    std::vector<fs::path> imgLeftPaths, imgRightPaths;
    for (fs::directory_iterator itr(imgsLeftPath); itr != fs::directory_iterator(); ++itr) {
        imgLeftPaths.push_back(itr->path());
    }
    std::sort(imgLeftPaths.begin(), imgLeftPaths.end());
    for (fs::directory_iterator itr(imgsRightPath); itr != fs::directory_iterator(); ++itr) {
        imgRightPaths.push_back(itr->path());
    }
    std::sort(imgRightPaths.begin(), imgRightPaths.end());

    std::cout << "Testing using " << imgLeftPaths.size() << " images" << std::endl;

    addImagesToCalibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>(
        calibrator,
        imgLeftPaths,
        imgRightPaths);

    ////
    /// Add IMU measurements
    ////

    std::vector<fs::path> imuPaths;
    for (fs::directory_iterator itr(imusPath); itr != fs::directory_iterator(); ++itr) {
        imuPaths.push_back(itr->path());
    }
    std::sort(imuPaths.begin(), imuPaths.end());

    std::cout << "Testing using " << imuPaths.size() << " IMU measurements" << std::endl;

    addImuToCalibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>(
        calibrator,
        imuPaths);
    ////
    /// Try to calibrate - this is the actual test
    ////

    calibrator.stopCollecting();

    calibrator.print(std::cout);

    auto intrResult = calibrator.calibrateCameraIntrinsics();

    EXPECT_TRUE(intrResult.has_value());
    if (!intrResult.has_value()) {
        return;
    }
    const auto res = intrResult.value();

    CameraCalibrationUtils::printResult(res[0], std::cout);
    CameraCalibrationUtils::printResult(res[1], std::cout);

    const auto cameraCalib0 = res[0];
    const auto distortion0 = cameraCalib0.distortion;

    EXPECT_NEAR(distortion0[0], -0.319038, 0.001);
    EXPECT_NEAR(distortion0[1], 0.0971273, 0.001);
    EXPECT_NEAR(distortion0[2], 0.000252449, 0.001);
    EXPECT_NEAR(distortion0[3], 0.000304249, 0.001);

    const auto cameraCalib1 = res[1];
    const auto distortion1 = cameraCalib1.distortion;

    EXPECT_NEAR(distortion1[0], -0.319845, 0.001);
    EXPECT_NEAR(distortion1[1], 0.0958489, 0.001);
    EXPECT_NEAR(distortion1[2], 0.000538449, 0.001);
    EXPECT_NEAR(distortion1[3], 0.000623019, 0.001);

    std::cout << "Baseline: " << cameraCalib0.baseline << std::endl;
    // EXPECT_LT(cameraCalib0.baseline)

    // CAM-IMU calibration

    calibrator.buildProblem();

    calibrator.getDvInfoBeforeOptimization(std::cout);

    try {
        const auto result = calibrator.calibrate();
        calibrator.getDvInfoAfterOptimization(std::cout);

        IccCalibratorUtils::printResult(result, std::cout);
        EXPECT_TRUE(result.converged);
    } catch (std::exception& ex) {
        EXPECT_TRUE(false);
        std::cout << ex.what() << std::endl;
        std::cout << "Optimization failed. Please make sure that the pattern is detected on all frames in your "
                     "dataset and repeat the calibration"
                  << std::endl;
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
