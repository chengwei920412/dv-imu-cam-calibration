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
		calibrator.addImu(singleImu.timestamp, singleImu.gyroscopeX, singleImu.gyroscopeY, singleImu.gyroscopeZ, singleImu.accelerometerX, singleImu.accelerometerY, singleImu.accelerometerZ);
	  }
	}

	// Process frame input
	auto frameInput = inputs.getFrameInput("frames");
	if (auto frame = frameInput.data()) {
	  cv::Mat img = *frame.getMatPointer();
	  calibrator.addImage(img, frame.timestamp());
	}

	// Output preview image
	auto preview = calibrator.getPreviewImage();
	outputs.getFrameOutput("preview") << preview.timestamp << preview.image << dv::commit;

  }
};

registerModuleClass(ImuCamCalibration)
