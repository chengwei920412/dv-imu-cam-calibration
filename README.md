# IMU camera calibration

DV module estimating the transformation and time shift between the camera and IMU.
This code is based on [kalibr](https://github.com/ethz-asl/kalibr).

## Build dependencies
* boost
* eigen
* libsuitesparse-de

## Usage
* calibrate your camea using DV [calibration](https://inivation.gitlab.io/dv/dv-docs/docs/tutorial-calibration/)
* adjust the module configuration
  * adjust your camera input IMU rate for BOTH calibration module AND camera input module (recommended 200Hz)
  * set the path to your DV camera calibration file
  * configure IMU noise parameters (defaults are valid for DVXplorer)
  * set the calibration pattern
  * restart the module to make sure that the changes are applied
  * connect IMU and accumulated frames inputs
    * recommended accumulator values - linear decay, 1e-8 decay coeff, 0.35 event contrib, decay at frame gen time, min/neutral/max potential 0.0/1.0/1.0
* collect images of a static calibration pattern while moving the camera 
    * you should record about 60s of data with calibration pattern visible on all frames
    * tilt camera 3x about each axis (left/right, up/down, clockwise/counterclockwise)
    * move camera 3x about each axis (left/right, up/down, front/back)
    * perform some random motion about all axes
* when collected enough data click calibrate and wait for the results to appear  

## Project structure
- modules - the DV module
- tests - smoke test of the module and test files
- utilities - Calibrator class which handles the camera calibration  
- kalibr_common, kalibr_imu_camera_calibration - scripts calibrating camera, based on kalibr Python equivalents. You can find them in the kalibr source code by searching for the file name.
- thirdparty - source code thirdparty libraries with small modifications 


## IMU noise parameters

In this section we provide the IMU noise parameters for the most common IMU sensors. More information about the IMU 
noise model can be found [here](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model).

### DVXplorer

Parameter | Value
--------- | -----
accelerometer noise density |  1.49e-3
acceletometer random walk | 8.69e-5
gyroscope noise density | 8.09e-5
gyroscope random walk | 2.29e-6

### DAVIS

Parameter | Value
--------- | -----
accelerometer noise density | 0.002  
acceletometer random walk | 4.0e-5
gyroscope noise density | 0.00018
gyroscope random walk | 0.001