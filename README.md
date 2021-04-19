# IMU camera calibration

DV module estimating the transformation and time shift between the camera and IMU.
This code is based on [kalibr](https://github.com/ethz-asl/kalibr).

## Build dependencies
- boost
- eigen
- libsuitesparse-dev

## Usage
- calibrate your camera using DV [calibration](https://inivation.gitlab.io/dv/dv-docs/docs/tutorial-calibration/)
- adjust your camera input IMU rate (recommended 200Hz) 
- add [imu parameters](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model) to your calibration file
  ```
  <!-- BMI160 DVXplorer -->
  <update_rate>200.0</update_rate>
  <acc_noise_density>1.49e-3</acc_noise_density>
  <acc_random_walk>8.69e-5</acc_random_walk>
  <gyr_noise_density>8.09e-5</gyr_noise_density>
  <gyr_random_walk>2.29e-6</gyr_random_walk>
  ```
- calibrate

## Project structure
- modules - the DV module
- tests - smoke test of the module and test files
- utilities - Calibrator class which handles the camera calibration  
- kalibr_common, kalibr_imu_camera_calibration - scripts calibrating camera, based on kalibr Python equivalents. You can find them in the kalibr source code by searching for the file name.
- thirdparty - source code thirdparty libraries with small modifications 

