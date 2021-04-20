#include <utilities/calibrator.hpp>

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

namespace fs = boost::filesystem;

int64_t str2int(std::string str) {
    char* pEnd;
    return std::strtoll(str.c_str(), &pEnd, 10);
}

TEST(CalibratorTestSuite, smokeTest) {
    ////
    /// Prepare the calibrator
    ////
    Calibrator::Options options;
    options.cols = 6;
    options.rows = 6;
    options.spacingMeters = 0.018;
    options.tagSpacing = 0.3;
    options.pattern = Calibrator::CalibrationPattern::APRIL_GRID;
    Calibrator calibrator(options);
    calibrator.startCollecting();

    ////
    /// Useful paths
    ////

    fs::path thisPath(__FILE__);
    fs::path imgsPath = thisPath.parent_path() / "test_files" / "img";
    fs::path imusPath = thisPath.parent_path() / "test_files" / "imu";

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
        imgPaths = std::vector<fs::path>(imgPaths.begin() + startIdx, imgPaths.begin() + startIdx + nIdx);
    }

    std::cout << "Testing using " << imgPaths.size() << " images" << std::endl;

    for (const auto& path : imgPaths) {
        cv::Mat img = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
        int64_t ts = str2int(path.stem().string());
        calibrator.addImage(img, ts);
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

        calibrator.addImu(ts, gX, gY, gZ, aX, aY, aZ);
    }

    ////
    /// Try to calibrate - this is the actual test
    ////

    calibrator.buildProblem();
    const auto result = calibrator.calibrate();

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.t_cam_imu, 0.00454318, 1e-8);

    Eigen::Matrix4d expected;
    expected << -0.999994, 0.000329469, 0.00342636, 0.000565281, 0.000306871, 0.999978, -0.00659382, 0.0191179,
        -0.00342846, -0.00659273, -0.999972, -0.0501339, 0, 0, 0, 1;
    EXPECT_TRUE(result.T_cam_imu.isApprox(expected, 1e-6)) << "result:" << std::endl
                                                           << result.T_cam_imu << std::endl
                                                           << "expected:" << std::endl
                                                           << expected;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
