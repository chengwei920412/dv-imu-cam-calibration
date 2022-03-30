
#include <dv-sdk/module.hpp>

#include <dv-processing/data/timed_keypoint_base.hpp>
#include <dv-processing/visualization/colors.hpp>
#include <regex>

class MarkersDrawing : public dv::ModuleBase {
protected:
    std::shared_ptr<const dv::Frame> latestFrame = nullptr;

    dv::cvector<dv::TimedKeyPoint> keypointBuffer;

public:
    static void initInputs(dv::InputDefinitionList& in) {
        in.addFrameInput("frames");
        in.addInput("markers", dv::TimedKeyPointPacketIdentifier());
    }

    static void initOutputs(dv::OutputDefinitionList& out) {
        out.addFrameOutput("preview");
    }

    static void initTypes(std::vector<dv::Types::Type>& types) {
        types.push_back(
            dv::Types::makeTypeDefinition<dv::TimedKeyPointPacket, dv::TimedKeyPoint>("KeyPoint with timestamp"));
    }

    static const char* initDescription() {
        return ("Draw marker detections on a frame");
    }

    static void initConfigOptions(dv::RuntimeConfig& config) {
    }

    MarkersDrawing() {
        outputs.getFrameOutput("preview").setup(inputs.getFrameInput("frames"));
    }

    void drawMarkers() {
        // Prepare image
        cv::Mat preview;
        if (latestFrame->image.channels() == 3) {
            latestFrame->image.copyTo(preview);
        } else {
            cv::cvtColor(latestFrame->image, preview, cv::COLOR_GRAY2BGR);
        }

        // Draw markers if timestamp matches perfectly
        auto iter = keypointBuffer.begin();
        while (iter != keypointBuffer.end() && iter->timestamp <= latestFrame->timestamp) {
            if (iter->timestamp == latestFrame->timestamp) {
                cv::drawMarker(
                    preview,
                    cv::Point(std::floor(iter->pt.x), std::floor(iter->pt.y)),
                    dv::visualization::colors::someNeonColor(iter->class_id),
                    cv::MARKER_DIAMOND, 7, 3);
            }
            iter++;
        }

        // Clear the drawn keypoints
        if (iter != keypointBuffer.begin()) {
            keypointBuffer.erase(keypointBuffer.begin(), iter);
        }

        // Push the preview
        outputs.getFrameOutput("preview") << latestFrame->timestamp << preview << dv::commit;

        // Forget the image
        latestFrame = nullptr;
    }

    void run() override {
        if (!latestFrame) {
            // Get some frame
            auto frameInput = inputs.getFrameInput("frames");
            if (auto frame = frameInput.data()) {
                latestFrame = frame.getBasePointer();
            }
        } else {
            // Read until we get some keypoints further in future to the frame, this means we received all
            // keypoints for the latest frame
            if (keypointBuffer.empty() || keypointBuffer.back().timestamp <= latestFrame->timestamp) {
                // Read one keypoint packet
                auto markersInput = inputs.getInput<dv::TimedKeyPointPacket>("markers");
                if (auto markers = markersInput.data()) {
                    keypointBuffer.insert(keypointBuffer.end(), markers->elements.begin(), markers->elements.end());
                }
            } else {
                // Enter here when we received enough keypoints
                drawMarkers();
            }
        }
    }
};

registerModuleClass(MarkersDrawing)
