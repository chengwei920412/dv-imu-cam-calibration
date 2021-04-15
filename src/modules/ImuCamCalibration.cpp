#include <dv-sdk/module.hpp>

#include <utilities/calibrator.hpp>

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
	const auto inputSize = inputs.getFrameInput("frames").size();
	const auto description = inputs.getFrameInput("frames").getOriginDescription();
	outputs.getFrameOutput("preview").setup(inputSize.width, inputSize.height, description);

	// Calibrator options
	Calibrator::Options options;

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

	// TODO(radam): IMU parameters

	calibrator = std::make_unique<Calibrator>(options);
  }

  void run() override {
    // Process IMU input
	auto imuInput = inputs.getIMUInput("imu");
	if (auto imuData = imuInput.data()) {
	  for (const auto &singleImu : imuData) {

//			// TODO(radam): del
					std::stringstream ss;
				ss << "/tmp/imu_" << singleImu.timestamp << ".txt";
				std::ofstream myfile(ss.str());
				myfile << singleImu.timestamp << std::endl;
				myfile << singleImu.gyroscopeX << std::endl;
				myfile <<  singleImu.gyroscopeY<< std::endl;
				myfile <<  singleImu.gyroscopeZ<< std::endl;
				myfile <<  singleImu.accelerometerX<< std::endl;
				myfile <<  singleImu.accelerometerY<< std::endl;
				myfile <<  singleImu.accelerometerZ << std::endl;




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

//	  	  // TODO(radam): del

			  	  std::stringstream ss;
	  	  ss << "/tmp/" << frame.timestamp() << ".png";
	  	  cv::imwrite(ss.str(), img);

	  // Output preview image
	  auto preview = calibrator->getPreviewImage();
	  outputs.getFrameOutput("preview") << preview.timestamp << preview.image << dv::commit;
	}

  }
};

registerModuleClass(ImuCamCalibration)
