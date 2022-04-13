#include "utilities/Calibrator.hpp"

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

namespace fs = boost::filesystem;

int64_t str2int(const std::string& str) {
    char* pEnd;
    return std::strtoll(str.c_str(), &pEnd, 10);
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

    //    images.reserve(imgPaths.size());
    for (size_t i = 0; i < imgLeftPaths.size(); i++) {
        CalibratorUtils::StampedImage imageLeft, imageRight;
        cv::Mat imgLeft = cv::imread(imgLeftPaths[i].string(), cv::IMREAD_GRAYSCALE);
        cv::Mat imgRight = cv::imread(imgRightPaths[i].string(), cv::IMREAD_GRAYSCALE);
        int64_t tsLeft = str2int(imgLeftPaths[i].stem().string());
        int64_t tsRight = str2int(imgRightPaths[i].stem().string());
        if (imgLeft.channels() == 3) {
            cv::Mat gray;
            cv::cvtColor(imgLeft, gray, cv::COLOR_BGR2GRAY);
            imageLeft = CalibratorUtils::StampedImage(gray, tsLeft);
        } else {
            imageLeft = CalibratorUtils::StampedImage(imgLeft, tsLeft);
        }
        if (imgRight.channels() == 3) {
            cv::Mat gray;
            cv::cvtColor(imgRight, gray, cv::COLOR_BGR2GRAY);
            imageRight = CalibratorUtils::StampedImage(gray, tsLeft);
        } else {
            imageRight = CalibratorUtils::StampedImage(imgRight, tsLeft);
        }
        calibrator.addImages({imageLeft, imageRight});
    }

    ////
    /// Add IMU measurements
    ////

    std::vector<fs::path> imuPaths;
    for (fs::directory_iterator itr(imusPath); itr != fs::directory_iterator(); ++itr) {
        imuPaths.push_back(itr->path());
    }
    std::sort(imuPaths.begin(), imuPaths.end());

    std::cout << "Testing using " << imuPaths.size() << " IMU measurements" << std::endl;

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

    ////
    /// Try to calibrate - this is the actual test
    ////

    calibrator.stopCollecting();

    auto intrResult = calibrator.calibrateCameraIntrinsics();
    EXPECT_TRUE(intrResult.has_value());
    if (!intrResult.has_value()) {
        return;
    }
    const auto res = intrResult.value();
    calibrator.print(std::cout);

    calibrator.buildProblem();
    const auto result = calibrator.calibrate();
    IccCalibratorUtils::printResult(result, std::cout);

    EXPECT_TRUE(result.converged);
}

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

    //    images.reserve(imgPaths.size());
    for (size_t i = 0; i < imgLeftPaths.size(); i++) {
        CalibratorUtils::StampedImage imageLeft, imageRight;
        cv::Mat imgLeft = cv::imread(imgLeftPaths[i].string(), cv::IMREAD_GRAYSCALE);
        cv::Mat imgRight = cv::imread(imgRightPaths[i].string(), cv::IMREAD_GRAYSCALE);
        int64_t tsLeft = str2int(imgLeftPaths[i].stem().string());
        int64_t tsRight = str2int(imgRightPaths[i].stem().string());
        if (imgLeft.channels() == 3) {
            cv::Mat gray;
            cv::cvtColor(imgLeft, gray, cv::COLOR_BGR2GRAY);
            imageLeft = CalibratorUtils::StampedImage(gray, tsLeft);
        } else {
            imageLeft = CalibratorUtils::StampedImage(imgLeft, tsLeft);
        }
        if (imgRight.channels() == 3) {
            cv::Mat gray;
            cv::cvtColor(imgRight, gray, cv::COLOR_BGR2GRAY);
            imageRight = CalibratorUtils::StampedImage(gray, tsLeft);
        } else {
            imageRight = CalibratorUtils::StampedImage(imgRight, tsLeft);
        }
        calibrator.addImages({imageLeft, imageRight});
    }

    ////
    /// Add IMU measurements
    ////

    std::vector<fs::path> imuPaths;
    for (fs::directory_iterator itr(imusPath); itr != fs::directory_iterator(); ++itr) {
        imuPaths.push_back(itr->path());
    }
    std::sort(imuPaths.begin(), imuPaths.end());

    std::cout << "Testing using " << imuPaths.size() << " IMU measurements" << std::endl;

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

    ////
    /// Try to calibrate - this is the actual test
    ////

    calibrator.stopCollecting();

    auto intrResult = calibrator.calibrateCameraIntrinsics();
    EXPECT_TRUE(intrResult.has_value());
    if (!intrResult.has_value()) {
        return;
    }
    const auto res = intrResult.value();
    calibrator.print(std::cout);

    calibrator.buildProblem();
    const auto result = calibrator.calibrate();
    IccCalibratorUtils::printResult(result, std::cout);

    EXPECT_TRUE(result.converged);
}

TEST(CalibratorTest, EquidistantDistortionModel) {
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
    options.maxIter = 50;
    options.timeCalibration = false;
    options.imuParameters.updateRate = 200.0;

    Calibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>
        calibrator(options);

    calibrator.startCollecting();

    ////
    /// Useful paths
    ////

    fs::path thisPath(__FILE__);
    fs::path imgsPath = thisPath.parent_path() / "test_files" / "1.98-1-1.8-fisheye" / "img_left";
    fs::path imusPath = thisPath.parent_path() / "test_files" / "1.98-1-1.8-fisheye" / "imu";

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

    //    images.reserve(imgPaths.size());
    for (const auto& path : imgPaths) {
        CalibratorUtils::StampedImage images;
        cv::Mat img = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
        int64_t ts = str2int(path.stem().string());
        if (img.channels() == 3) {
            cv::Mat gray;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
            images=CalibratorUtils::StampedImage(gray, ts);
        } else {
            images=CalibratorUtils::StampedImage(img, ts);
        }
        calibrator.addImages({images});
    }

    ////
    /// Add IMU measurements
    ////

    std::vector<fs::path> imuPaths;
    for (fs::directory_iterator itr(imusPath); itr != fs::directory_iterator(); ++itr) {
        imuPaths.push_back(itr->path());
    }
    std::sort(imuPaths.begin(), imuPaths.end());

    std::cout << "Testing using " << imuPaths.size() << " IMU measurements" << std::endl;

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

    ////
    /// Try to calibrate - this is the actual test
    ////

    calibrator.stopCollecting();

    auto intrResult = calibrator.calibrateCameraIntrinsics();
    EXPECT_TRUE(intrResult.has_value());
    if (!intrResult.has_value()) {
        return;
    }
    const auto res = intrResult.value();
    calibrator.print(std::cout);
    //    EXPECT_NEAR(res.projection.size(), 4, 1e-8);
    //    EXPECT_NEAR(res.projection.at(0), 638.0032149263036, 1e-8);
    //    EXPECT_NEAR(res.projection.at(1), 638.63729649198285, 1e-8);
    //    EXPECT_NEAR(res.projection.at(2), 290.11498231898963, 1e-8);
    //    EXPECT_NEAR(res.projection.at(3), 256.74408382366255, 1e-8);
    //    EXPECT_NEAR(res.distortion.size(), 4, 1e-8);
    //    EXPECT_NEAR(res.distortion.at(0), -0.38064277580652356, 1e-8);
    //    EXPECT_NEAR(res.distortion.at(1), 0.26161769580979349, 1e-8);
    //    EXPECT_NEAR(res.distortion.at(2), 0.000300625, 1e-8);
    //    EXPECT_NEAR(res.distortion.at(3), -0.00161274, 1e-8);

    calibrator.buildProblem();
    const auto result = calibrator.calibrate();
    IccCalibratorUtils::printResult(result, std::cout);

    EXPECT_TRUE(result.converged);
    //    EXPECT_NEAR(result.t_cam_imu, -0.00039430212860192481, 1e-8);

    //     Eigen::Matrix4d expected;
    //     expected << -0.99997614684923397, -0.0066787868433113226, -0.001760550726610285, -0.00088461239053432906,
    //         -0.0066860030983353844, 0.99996913895061124, 0.0041253495539700716, 0.01810628127255811,
    //         0.0017329440638422437, 0.0041370221989980912, -0.99998994092550586, -0.041554955113069315, 0, 0, 0, 1;
    //     EXPECT_TRUE(result.T_cam_imu.isApprox(expected, 1e-6)) << "result:" << std::endl
    //                                                            << result.T_cam_imu << std::endl
    //                                                            << "expected:" << std::endl
    //                                                            << expected;
}

 TEST(CalibratorTest, RadTanDistortionModel) {
     ////
     /// Prepare the calibrator
     ////
     CalibratorUtils::Options options;
     options.cols = 6;
     options.rows = 6;
     options.spacingMeters = 0.018;
     options.patternSpacing = 0.3;
     options.pattern = CalibratorUtils::PatternType::APRIL_GRID;
     options.cameraInitialSettings.emplace_back().imageSize = cv::Size(640, 480);
     options.maxIter = 50;
     options.timeCalibration = false;
     options.imuParameters.updateRate = 200.0;

     Calibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>
     calibrator(options); calibrator.startCollecting();

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

     //    images.reserve(imgPaths.size());
     for (const auto& path : imgPaths) {
         CalibratorUtils::StampedImage images;
         cv::Mat img = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
         int64_t ts = str2int(path.stem().string());
         if (img.channels() == 3) {
             cv::Mat gray;
             cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
             images=CalibratorUtils::StampedImage(gray, ts);
         } else {
             images=CalibratorUtils::StampedImage(img, ts);
         }
         calibrator.addImages({images});
     }

     ////
     /// Add IMU measurements
     ////

     std::vector<fs::path> imuPaths;
     for (fs::directory_iterator itr(imusPath); itr != fs::directory_iterator(); ++itr) {
         imuPaths.push_back(itr->path());
     }
     std::sort(imuPaths.begin(), imuPaths.end());

     std::cout << "Testing using " << imuPaths.size() << " IMU measurements" << std::endl;

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

     ////
     /// Try to calibrate - this is the actual test
     ////

     calibrator.stopCollecting();

     auto intrResult = calibrator.calibrateCameraIntrinsics();
     EXPECT_TRUE(intrResult.has_value());
     if (!intrResult.has_value()) {
         return;
     }
     const auto res = intrResult.value();
     calibrator.print(std::cout);

     calibrator.buildProblem();
     const auto result = calibrator.calibrate();
     IccCalibratorUtils::printResult(result, std::cout);

     EXPECT_TRUE(result.converged);
     //    EXPECT_NEAR(result.t_cam_imu, -0.00039430212860192481, 1e-8);

//     Eigen::Matrix4d expected;
//     expected << -0.99997614684923397, -0.0066787868433113226, -0.001760550726610285, -0.00088461239053432906,
//         -0.0066860030983353844, 0.99996913895061124, 0.0041253495539700716, 0.01810628127255811,
//         0.0017329440638422437, 0.0041370221989980912, -0.99998994092550586, -0.041554955113069315, 0, 0, 0, 1;
//     EXPECT_TRUE(result.T_cam_imu.isApprox(expected, 1e-6)) << "result:" << std::endl
//                                                            << result.T_cam_imu << std::endl
//                                                            << "expected:" << std::endl
//                                                            << expected;
 }

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
