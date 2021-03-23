#include <dv-sdk/module.hpp>

#include <utilities/calibrator.hpp>

class ImuCamCalibration : public dv::ModuleBase {
private:
  Calibrator calibrator;

public:
  static void initInputs(dv::InputDefinitionList &in) {
	in.addFrameInput("frames");
  }

  static void initOutputs(dv::OutputDefinitionList &out) {
	out.addFrameOutput("frames");
  }

  static const char *initDescription() {
	return ("Calibrate the IMU with respect to the event camera.");
  }

  static void initConfigOptions(dv::RuntimeConfig &config) {
  }

  ImuCamCalibration() {
	const auto inputSize = inputs.getFrameInput("frames").size();
	const auto description = inputs.getEventInput("frames").getOriginDescription();
	outputs.getFrameOutput("frames").setup(inputSize.width, inputSize.height, description);
  }

  void run() override {

	auto frame = inputs.getFrameInput("frames").frame();
	cv::Mat img = *frame.getMatPointer();
	img = calibrator.process(img);
	outputs.getFrameOutput("frames") << frame.timestamp() << img << dv::commit;

  }
};

registerModuleClass(ImuCamCalibration)
