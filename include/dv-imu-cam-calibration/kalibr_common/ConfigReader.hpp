//
// Created by radam on 2021-03-23.
//

#pragma once

#include <aslam/Frame.hpp>
#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/cameras.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/EquidistantDistortion.hpp>
#include <aslam/cameras/PinholeProjection.hpp>

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

#include <vector>

template<typename CameraGeometryType, typename DistortionType>
class CameraModel {
protected:
    boost::shared_ptr<CameraGeometryType> geometry = nullptr;

    std::vector<double> focalLength;
    std::vector<double> principalPoint;
    std::vector<double> distortionCoefficients;

public:
    CameraModel(
        const std::vector<double>& intrinsics,
        const std::vector<double>& distCoeff,
        const cv::Size& resolution) {
        assert(intrinsics.size() == 4);
        focalLength = std::vector<double>(intrinsics.begin(), intrinsics.begin() + 2);
        principalPoint = std::vector<double>(intrinsics.begin() + 2, intrinsics.begin() + 4);

        if constexpr (
            std::is_same<CameraGeometryType, aslam::cameras::EquidistantDistortedPinholeCameraGeometry>()
            && std::is_same<DistortionType, aslam::cameras::EquidistantDistortion>()) {
            std::cout << "Camera Model: Equidistant Distorted Pinhole\nDistortion Model: Equidistant" << std::endl;
        }
        if constexpr (
            std::is_same<CameraGeometryType, aslam::cameras::DistortedPinholeCameraGeometry>()
            && std::is_same<DistortionType, aslam::cameras::RadialTangentialDistortion>()) {
            std::cout << "Camera Model: Distorted Pinhole \nDistortion Model: RadialTangential" << std::endl;
        }
        if constexpr (
            std::is_same<CameraGeometryType, aslam::cameras::FovDistortedPinholeCameraGeometry>()
            && std::is_same<DistortionType, aslam::cameras::FovDistortion>()) {
            std::cout << "Camera Model: Distorted Pinhole Fov \nDistortion Model: FovDistortion" << std::endl;
        }

        distortionCoefficients = distCoeff;
        assert(
            (std::is_same<CameraGeometryType, aslam::cameras::FovDistortedPinholeCameraGeometry>()
             && distortionCoefficients.size() == 1)
            || (!std::is_same<CameraGeometryType, aslam::cameras::FovDistortedPinholeCameraGeometry>()
                && distortionCoefficients.size() >= 4));

        DistortionType dist;
        if constexpr (
            std::is_same<CameraGeometryType, aslam::cameras::FovDistortedPinholeCameraGeometry>()
            && std::is_same<DistortionType, aslam::cameras::FovDistortion>()) {
            std::cout << "Camera Model: Distorted Pinhole Fov \nDistortion Model: FovDistortion" << std::endl;
            dist = aslam::cameras::FovDistortion(distortionCoefficients.at(0));
        } else {
            dist = DistortionType(
                distortionCoefficients.at(0),
                distortionCoefficients.at(1),
                distortionCoefficients.at(2),
                distortionCoefficients.at(3));
        }
        const auto proj = aslam::cameras::PinholeProjection<DistortionType>(
            focalLength[0],
            focalLength[1],
            principalPoint[0],
            principalPoint[1],
            resolution.width,
            resolution.height,
            dist);

        geometry = boost::make_shared<CameraGeometryType>(proj);
    }

    boost::shared_ptr<CameraGeometryType> getGeometry() {
        return geometry;
    }

    boost::shared_ptr<aslam::Frame<CameraGeometryType>> frame() {
        auto frame = boost::make_shared<aslam::Frame<CameraGeometryType>>(aslam::Frame<CameraGeometryType>());
        return frame;
    }

    void printDetails() {
        std::cout << "Initializing camera:" << std::endl;
        std::cout << "  Camera model: pinhole" << std::endl;
        std::cout << "  Focal length: [" << focalLength.at(0) << " " << focalLength.at(1) << "]" << std::endl;
        std::cout << "  Principal point: [" << principalPoint.at(0) << " " << principalPoint.at(1) << "]" << std::endl;
        if constexpr (std::is_same<CameraGeometryType, aslam::cameras::EquidistantDistortedPinholeCameraGeometry>()) {
            std::cout << "  Distortion model: Equidistant" << std::endl;
        } else {
            std::cout << "  Distortion model: RadialTangential" << std::endl;
        }
        if constexpr (std::is_same<CameraGeometryType, aslam::cameras::FovDistortedPinholeCameraGeometry>()) {
            std::cout << "  Distortion coefficients: [" << distortionCoefficients.at(0) << "]" << std::endl;
        } else {
            std::cout << "  Distortion coefficients: [" << distortionCoefficients.at(0) << " "
                      << distortionCoefficients.at(1) << " " << distortionCoefficients.at(2) << " "
                      << distortionCoefficients.at(3) << "]" << std::endl;
        }
    }

    std::ostream& print(std::ostream& os) {
        os << "Initializing camera:" << std::endl;
        os << "  Camera model: pinhole" << std::endl;
        os << "  Focal length: [" << focalLength.at(0) << " " << focalLength.at(1) << "]" << std::endl;
        os << "  Principal point: [" << principalPoint.at(0) << " " << principalPoint.at(1) << "]"
           << std::endl;
        os << "  Distortion model: RadialTangential" << std::endl;
        os << "  Distortion coefficients: [" << distortionCoefficients.at(0) << " "
           << distortionCoefficients.at(1) << " " << distortionCoefficients.at(2) << " "
           << distortionCoefficients.at(3) << "]" << std::endl;
        return os;
    }
};

struct ImuParameters {
    double updateRate = 200.0;
    double accNoiseDensity = 1.4e-3;
    double accRandomWalk = 8.6e-5;
    double gyrNoiseDensity = 8.0e-5;
    double gyrRandomWalk = 2.2e-6;

    std::tuple<double, double, double> getAccelerometerStatistics() const {
        double accelUncertaintyDiscrete = accNoiseDensity / sqrt(1.0 / updateRate);
        return std::make_tuple(accelUncertaintyDiscrete, accRandomWalk, accNoiseDensity);
    }

    std::tuple<double, double, double> getGyroStatistics() const {
        double gyroUncertaintyDiscrete = gyrNoiseDensity / sqrt(1.0 / updateRate);
        return std::make_tuple(gyroUncertaintyDiscrete, gyrRandomWalk, gyrNoiseDensity);
    }
};
