//
// Created by radam on 2021-03-23.
//

#include <kalibr_common/ConfigReader.hpp>

#include <aslam/cameras.hpp>

PinholeRadialTangentialCamera::PinholeRadialTangentialCamera(
    const std::vector<double>& intrinsics,
    const std::vector<double>& distCoeff,
    const cv::Size& resolution) {
    assert(intrinsics.size() == 4);
    focalLength = std::vector<double>(intrinsics.begin(), intrinsics.begin() + 2);
    principalPoint = std::vector<double>(intrinsics.begin() + 2, intrinsics.begin() + 4);

    distortionCoefficients = distCoeff;
    assert(distortionCoefficients.size() >= 4);
    const auto dist = aslam::cameras::RadialTangentialDistortion(
        distortionCoefficients.at(0),
        distortionCoefficients.at(1),
        distortionCoefficients.at(2),
        distortionCoefficients.at(3));
    const auto proj = aslam::cameras::PinholeProjection<aslam::cameras::RadialTangentialDistortion>(
        focalLength[0],
        focalLength[1],
        principalPoint[0],
        principalPoint[1],
        resolution.width,
        resolution.height,
        dist);

    geometry = boost::make_shared<aslam::cameras::DistortedPinholeCameraGeometry>(proj);
}

boost::shared_ptr<aslam::cameras::DistortedPinholeCameraGeometry> PinholeRadialTangentialCamera::getGeometry() {
    return geometry;
}

boost::shared_ptr<aslam::Frame<aslam::cameras::DistortedPinholeCameraGeometry>> PinholeRadialTangentialCamera::frame() {
    auto frame = boost::make_shared<aslam::Frame<aslam::cameras::DistortedPinholeCameraGeometry>>(
        aslam::Frame<aslam::cameras::DistortedPinholeCameraGeometry>());
    return frame;
}

void PinholeRadialTangentialCamera::printDetails() {
    std::cout << "Initializing camera:" << std::endl;
    std::cout << "  Camera model: pinhole" << std::endl;
    std::cout << "  Focal length: [" << focalLength.at(0) << " " << focalLength.at(1) << "]" << std::endl;
    std::cout << "  Principal point: [" << principalPoint.at(0) << " " << principalPoint.at(1) << "]" << std::endl;
    std::cout << "  Distortion model: RadialTangential" << std::endl;
    std::cout << "  Distortion coefficients: [" << distortionCoefficients.at(0) << " " << distortionCoefficients.at(1)
              << " " << distortionCoefficients.at(2) << " " << distortionCoefficients.at(3) << "]" << std::endl;
}