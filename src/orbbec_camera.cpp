#include "aerial_touch/orbbec_camera.hpp"
#include "aerial_touch/alignment_mode.hpp"

#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Utils.hpp>

#include <algorithm>
#include <cstring>
#include <exception>

namespace aerial_touch {
namespace {

std::shared_ptr<ob::Config> hardware_align_config(const std::shared_ptr<ob::Pipeline>& pipeline) {
    const auto color_profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    const auto color_count = color_profiles->getCount();
    for(std::uint32_t color_index = 0; color_index < color_count; ++color_index) {
        const auto color_profile = color_profiles->getProfile(color_index);
        const auto color_video = color_profile->as<ob::VideoStreamProfile>();
        if(color_video->getFormat() != OB_FORMAT_RGB) {
            continue;
        }

        std::shared_ptr<ob::StreamProfileList> depth_profiles;
        try {
            depth_profiles = pipeline->getD2CDepthProfileList(color_profile, ALIGN_D2C_HW_MODE);
        }
        catch(const ob::Error&) {
            // Gemini 2 firmware without the optional hardware D2C processor must use ob::Align instead.
            return nullptr;
        }
        const auto depth_count = depth_profiles->getCount();
        for(std::uint32_t depth_index = 0; depth_index < depth_count; ++depth_index) {
            const auto depth_profile = depth_profiles->getProfile(depth_index);
            const auto depth_video = depth_profile->as<ob::VideoStreamProfile>();
            if(depth_video->getFps() != color_video->getFps()) {
                continue;
            }

            auto config = std::make_shared<ob::Config>();
            config->enableStream(color_profile);
            config->enableStream(depth_profile);
            config->setAlignMode(ALIGN_D2C_HW_MODE);
            config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
            return config;
        }
    }
    return nullptr;
}

std::shared_ptr<ob::Config> software_align_config() {
    auto config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_ANY);
    config->enableVideoStream(OB_STREAM_COLOR, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_RGB);
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    return config;
}

std::string describe_error(const ob::Error& error) {
    return std::string(error.what()) + " [function=" + error.getFunction() + ", args=" + error.getArgs() + "]";
}

}  // namespace

struct OrbbecCamera::Impl {
    std::shared_ptr<ob::Pipeline> pipeline;
    std::shared_ptr<ob::Align> software_aligner;
    bool running{ false };
    bool hardware_alignment{ false };
    std::string error;
};

OrbbecCamera::OrbbecCamera() : impl_(std::make_unique<Impl>()) {}

OrbbecCamera::~OrbbecCamera() {
    stop();
}

bool OrbbecCamera::start() {
    stop();
    impl_->error.clear();
    try {
        impl_->pipeline = std::make_shared<ob::Pipeline>();
        impl_->pipeline->enableFrameSync();

        auto config = hardware_align_config(impl_->pipeline);
        const auto alignment_mode = choose_alignment_mode(config != nullptr);
        impl_->hardware_alignment = alignment_mode == AlignmentMode::Hardware;
        if(alignment_mode == AlignmentMode::Software) {
            config = software_align_config();
            impl_->software_aligner = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            impl_->software_aligner->setMatchTargetResolution(true);
        }
        impl_->pipeline->start(config);
        impl_->running = true;
        return true;
    }
    catch(const ob::Error& error) {
        impl_->error = "Orbbec start failed: " + describe_error(error);
    }
    catch(const std::exception& error) {
        impl_->error = std::string("Orbbec start failed: ") + error.what();
    }
    impl_->pipeline.reset();
    impl_->software_aligner.reset();
    return false;
}

void OrbbecCamera::stop() {
    if(impl_ && impl_->pipeline && impl_->running) {
        try {
            impl_->pipeline->stop();
        }
        catch(...) {
        }
    }
    if(impl_) {
        impl_->running = false;
        impl_->pipeline.reset();
        impl_->software_aligner.reset();
    }
}

std::optional<RgbdFrame> OrbbecCamera::capture(const std::uint32_t timeout_ms) {
    if(!impl_->running || !impl_->pipeline) {
        impl_->error = "Orbbec camera is not running";
        return std::nullopt;
    }
    try {
        auto frameset = impl_->pipeline->waitForFrameset(timeout_ms);
        if(!frameset) {
            return std::nullopt;
        }
        if(impl_->software_aligner) {
            const auto aligned = impl_->software_aligner->process(frameset);
            if(!aligned) {
                return std::nullopt;
            }
            frameset = aligned->as<ob::FrameSet>();
        }

        const auto color = frameset->getColorFrame();
        const auto depth = frameset->getDepthFrame();
        if(!color || !depth || color->getFormat() != OB_FORMAT_RGB) {
            impl_->error = "Frame set is missing RGB or aligned depth data";
            return std::nullopt;
        }

        RgbdFrame frame;
        frame.color_width = static_cast<int>(color->getWidth());
        frame.color_height = static_cast<int>(color->getHeight());
        frame.depth_width = static_cast<int>(depth->getWidth());
        frame.depth_height = static_cast<int>(depth->getHeight());
        frame.timestamp_ms = static_cast<std::int64_t>(color->getTimeStampUs() / 1000U);
        frame.depth_unit_mm = depth->getValueScale();

        const auto rgb_bytes = static_cast<std::size_t>(frame.color_width) * static_cast<std::size_t>(frame.color_height) * 3U;
        const auto depth_values = static_cast<std::size_t>(frame.depth_width) * static_cast<std::size_t>(frame.depth_height);
        if(color->getDataSize() < rgb_bytes || depth->getDataSize() < depth_values * sizeof(std::uint16_t)) {
            impl_->error = "Frame buffers are smaller than their stream profiles";
            return std::nullopt;
        }
        frame.rgb.resize(rgb_bytes);
        frame.depth.resize(depth_values);
        std::memcpy(frame.rgb.data(), color->getData(), rgb_bytes);
        std::memcpy(frame.depth.data(), depth->getData(), depth_values * sizeof(std::uint16_t));

        const auto color_profile = color->getStreamProfile();
        const auto depth_profile = depth->getStreamProfile();
        const auto intrinsic = depth_profile->as<ob::VideoStreamProfile>()->getIntrinsic();
        const auto extrinsic = depth_profile->getExtrinsicTo(color_profile);
        frame.depth_intrinsics = { intrinsic.fx, intrinsic.fy, intrinsic.cx, intrinsic.cy, intrinsic.width, intrinsic.height };
        std::copy(std::begin(extrinsic.rot), std::end(extrinsic.rot), frame.depth_to_color.rotation.begin());
        std::copy(std::begin(extrinsic.trans), std::end(extrinsic.trans), frame.depth_to_color.translation_mm.begin());
        frame.profiles_valid = intrinsic.fx > 0.0F && intrinsic.fy > 0.0F;

        if(!frame.valid()) {
            impl_->error = "RGB and aligned depth frames do not share a valid pixel grid";
            return std::nullopt;
        }
        impl_->error.clear();
        return frame;
    }
    catch(const ob::Error& error) {
        impl_->error = "Orbbec capture failed: " + describe_error(error);
    }
    catch(const std::exception& error) {
        impl_->error = std::string("Orbbec capture failed: ") + error.what();
    }
    return std::nullopt;
}

std::optional<Vec3> OrbbecCamera::deproject(const RgbdFrame& frame, const Vec2 pixel, const float depth_mm) const {
    if(!frame.valid() || depth_mm <= 0.0F || pixel.x < 0.0F || pixel.y < 0.0F
       || pixel.x >= static_cast<float>(frame.depth_width) || pixel.y >= static_cast<float>(frame.depth_height)) {
        return std::nullopt;
    }

    OBCameraIntrinsic intrinsic{ frame.depth_intrinsics.fx, frame.depth_intrinsics.fy, frame.depth_intrinsics.cx,
                                 frame.depth_intrinsics.cy, static_cast<std::int16_t>(frame.depth_intrinsics.width),
                                 static_cast<std::int16_t>(frame.depth_intrinsics.height) };
    OBExtrinsic extrinsic{};
    std::copy(frame.depth_to_color.rotation.begin(), frame.depth_to_color.rotation.end(), std::begin(extrinsic.rot));
    std::copy(frame.depth_to_color.translation_mm.begin(), frame.depth_to_color.translation_mm.end(), std::begin(extrinsic.trans));
    const OBPoint2f source{ pixel.x, pixel.y };
    OBPoint3f target{};
    if(!ob::CoordinateTransformHelper::transformation2dto3d(source, depth_mm, intrinsic, extrinsic, &target)) {
        return std::nullopt;
    }
    return Vec3{ target.x, target.y, target.z };
}

bool OrbbecCamera::running() const {
    return impl_->running;
}

bool OrbbecCamera::hardware_alignment() const {
    return impl_->hardware_alignment;
}

const std::string& OrbbecCamera::error() const {
    return impl_->error;
}

}  // namespace aerial_touch
