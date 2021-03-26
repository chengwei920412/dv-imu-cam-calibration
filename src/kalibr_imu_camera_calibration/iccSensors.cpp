//
// Created by radam on 2021-03-25.
//

#include <kalibr_imu_camera_calibration/iccSensors.hpp>

IccCamera::IccCamera(const double reprojectionSigma,
					 const bool showCorners,
					 const bool showReproj,
					 const bool showOneStep) : cornerUncertainty(reprojectionSigma){

  targetObservations = false; // TODO(radam): pass these from constructor?
  gravity_w = Eigen::Vector3d(9.80655, 0., 0.);


}

