#include <dv-sdk/module.hpp>

#include <utilities/calibrator.hpp>

class ImuCamCalibration : public dv::ModuleBase {
private:
  Calibrator calibrator;

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
  }

  ImuCamCalibration() {
	const auto inputSize = inputs.getFrameInput("frames").size();
	const auto description = inputs.getFrameInput("frames").getOriginDescription();
	outputs.getFrameOutput("preview").setup(inputSize.width, inputSize.height, description);
  }

  void run() override {
    // Process IMU input
	auto imuInput = inputs.getIMUInput("imu");
	if (auto imuData = imuInput.data()) {
	  for (const auto &singleImu : imuData) {
		calibrator.addImu(singleImu.timestamp,
					static_cast<double>(singleImu.gyroscopeX) * M_PI / 180,
						static_cast<double>(singleImu.gyroscopeY) * M_PI / 180,
							static_cast<double>(singleImu.gyroscopeZ) * M_PI / 180,
								static_cast<double>(singleImu.accelerometerX) * 9.81,
									static_cast<double>(singleImu.accelerometerY) * 9.81,
										static_cast<double>(singleImu.accelerometerZ) * 9.81);
	  }
	}

	// Process frame input
	auto frameInput = inputs.getFrameInput("frames");
	if (auto frame = frameInput.data()) {
	  cv::Mat img = *frame.getMatPointer();
	  calibrator.addImage(img, frame.timestamp());


	  // Output preview image
	  auto preview = calibrator.getPreviewImage();
	  outputs.getFrameOutput("preview") << preview.timestamp << preview.image << dv::commit;
	}

	// TODO(radam): param
	if (calibrator.getNumDetections() > 900) {
	  calibrator.calibrate();
	  throw std::runtime_error("END"); // TODO(radam): delete
	}
  }
};

registerModuleClass(ImuCamCalibration)
