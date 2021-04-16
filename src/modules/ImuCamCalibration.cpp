#include <dv-sdk/module.hpp>

#include <utilities/calibrator.hpp>


bool cvExists(const cv::FileNode &fn) {
  if (fn.type() == cv::FileNode::NONE) {
	return (false);
  }

  return (true);
}

class ImuCamCalibration : public dv::ModuleBase {
private:
  std::unique_ptr<Calibrator> calibrator = nullptr;

public:
  static void initInputs(dv::InputDefinitionList &in) {
	in.addFrameInput("frames");
	in.addIMUInput("imu");
  }

  static void initOutputs(dv::OutputDefinitionList &out) {
	out.addFrameOutput("preview");
  }

  static const char *initDescription() {
	return ("Calibrate the IMU with respect to the event camera.");
  }

  static void initConfigOptions(dv::RuntimeConfig &config) {
    // Camera calibration file
    config.add("calibrationFile", dv::ConfigOption::fileOpenOption("Path to the camera calibration file", "", "xml"));

    // Calibration pattern
	config.add("boardHeight", dv::ConfigOption::intOption("Number of rows in the calibration pattern", 11, 1, 50));
	config.add("boardWidth", dv::ConfigOption::intOption("Number of cols in the calibration pattern", 4, 1, 50));
	config.add("boardSquareSize", dv::ConfigOption::doubleOption("Size of a calibration pattern element in meters", 0.05, 0.0, 1.0));
	config.add("tagSpacing", dv::ConfigOption::doubleOption("Ratio of space between tags to tagSize (AprilTag)", 0.05, 0.0, 1.0));
    config.add("calibrationPattern", dv::ConfigOption::listOption("Calibration pattern to use", "assCircleGrid", {"chessboard", "assCircleGrid", "aprilTag"}, false));

    // Module control buttons
    config.add("startCollecting", dv::ConfigOption::buttonOption("Begin collecting calibration images", "startCollecting"));
    config.add("calibrate", dv::ConfigOption::buttonOption("Start calibration algorithm", "calibrate"));

    // Optimization options
    config.add("maxIter", dv::ConfigOption::intOption("Maximum number of iteration of calibration optimization problem", 20, 1, 100));
    config.add("timeCalibration", dv::ConfigOption::boolOption("If true, time offset between the sensors will be calibrated", true));

	config.setPriorityOptions({"calibrationFile", "boardHeight", "boardWidth", "boardSquareSize", "calibrationPattern", "startCollecting", "calibrate" });
  }

  void configUpdate() {

    // Start collecting button was clicked
    if (config.getBool("startCollecting")) {
      calibrator->startCollecting();
	  config.setBool("startCollecting", false);
    }

    // Calibrate button was clicked
    if (config.getBool("calibrate")) {
      calibrator->calibrate();
      config.setBool("calibrate", false);
    }
  }

  ImuCamCalibration() {
    // Input output
    const auto frameInput = inputs.getFrameInput("frames");
	const auto inputSize = frameInput.size();
	const auto description = frameInput.getOriginDescription();
	outputs.getFrameOutput("preview").setup(inputSize.width, inputSize.height, description);


	// Calibrator options
	Calibrator::Options options;

	// Calibration file
	const auto filename = config.getString("calibrationFile");
	bool success = readCalibrationFile(filename, frameInput.getOriginDescription(), &options);

	if (!success) {
	  log.error << "Failed to load camera calibration file. Using default data. " << filename ;
	}

	// Calibration pattern
	const auto pattern = config.getString("calibrationPattern");
	if (pattern == "chessboard") {
	  options.pattern = Calibrator::CalibrationPattern::CHESSBOARD;
	} else if (pattern == "assCircleGrid"){
	  options.pattern = Calibrator::CalibrationPattern::ASYMMETRIC_CIRCLES_GRID;
	} else if (pattern == "aprilTag") {
	  options.pattern = Calibrator::CalibrationPattern::APRIL_GRID;
	} else {
	  std::stringstream  ss;
	  ss << "Unknown calibration pattern: " << pattern;
	  throw std::runtime_error(ss.str());
	}

	options.rows = static_cast<size_t>(config.getInt("boardHeight"));
	options.cols = static_cast<size_t>(config.getInt("boardWidth"));
	options.spacingMeters = config.getDouble("boardSquareSize");
	options.tagSpacing = config.getDouble("tagSpacing");

	options.maxIter = static_cast<size_t>(config.getInt("maxIter"));
    options.timeCalibration = config.getBool("timeCalibration");

	calibrator = std::make_unique<Calibrator>(options);
  }

  void run() override {
    // Process IMU input
	auto imuInput = inputs.getIMUInput("imu");
	if (auto imuData = imuInput.data()) {
	  for (const auto &singleImu : imuData) {
		calibrator->addImu(singleImu.timestamp,
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
  bool readCalibrationFile(const std::string& filename, const std::string& cameraID, Calibrator::Options* options) {
    assert(options);

	cv::FileStorage fs(filename, cv::FileStorage::READ);

	if (!fs.isOpened()) {
	  log.error << "Impossible to load the camera calibration file."  << std::endl;
	  return false;
	}

	const auto typeNode = fs["type"];

	if (!cvExists(typeNode) || !typeNode.isString() || (typeNode.string() != "camera")) {
	  log.error << "Wrong type of camera calibration file." << std::endl;
	  return false;
	}

	const auto useFisheye = fs["use_fisheye_model"];
	if (!cvExists(useFisheye) || !useFisheye.isInt()) {
      int fseye;
      useFisheye >> fseye;
      if (fseye != 0) {
		log.error << "Only non-fisheye calibration is supported!" << std::endl;
		return false;
      }
	}

  auto cameraNode = fs[cameraID];

  if (!cvExists(cameraNode) || !cameraNode.isMap()) {
	log.warning.format(
		"Calibration data for camera {:s} not present in file: {:s}", cameraID, filename);
	return false;
  }

  if (!cvExists(cameraNode["camera_matrix"]) || !cvExists(cameraNode["distortion_coefficients"])) {
	log.warning.format(
		"Calibration data for camera {:s} not present in file: {:s}", cameraID, filename);
	return false;
  }

  cv::Mat loadedCameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat loadedDistCoeffs   = cv::Mat::zeros(8, 1, CV_64F);
  int imWidth, imHeight;

  cameraNode["camera_matrix"] >> loadedCameraMatrix;
  cameraNode["distortion_coefficients"] >> loadedDistCoeffs;


  cameraNode["image_width"] >> imWidth;
  cameraNode["image_height"] >> imHeight;

  log.info.format("Loaded camera matrix and distortion coefficients for camera {:s} from file: {:s}", cameraID, filename);

  options->intrinsics = std::vector<double>{loadedCameraMatrix.at<double>(0,0),
											loadedCameraMatrix.at<double>(1,1),
											loadedCameraMatrix.at<double>(0,2),
											loadedCameraMatrix.at<double>(1,2)};

  options->distCoeffs = std::vector<double>{loadedDistCoeffs.at<double>(0,0),
											loadedDistCoeffs.at<double>(1,0),
											loadedDistCoeffs.at<double>(2,0),
											loadedDistCoeffs.at<double>(3,0)};
  options->imageSize = cv::Size(imWidth, imHeight);

  auto updateRate = cameraNode["update_rate"];
  if (cvExists(updateRate)) {
    updateRate >> options->imuParameters.updateRate;
  }

  auto accNoiseDensity = cameraNode["acc_noise_density"];
  if (cvExists(accNoiseDensity)) {
    accNoiseDensity >> options->imuParameters.accNoiseDensity;
  }

  auto accRandomWalk = cameraNode["acc_random_walk"];
  if (cvExists(accRandomWalk)) {
    accRandomWalk >> options->imuParameters.accRandomWalk;
  }

  auto gyrNoiseDensity = cameraNode["gyr_noise_density"];
  if (cvExists(gyrNoiseDensity)) {
    gyrNoiseDensity >> options->imuParameters.gyrNoiseDensity;
  }

  auto gyrRandomWalk = cameraNode["gyr_random_walk"];
  if (cvExists(gyrRandomWalk)) {
    gyrRandomWalk >> options->imuParameters.gyrRandomWalk;
  }

    return true;
  }
};

registerModuleClass(ImuCamCalibration)
