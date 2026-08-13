#include <libobsensor/ObSensor.hpp>

#include <iostream>
#include <memory>
#include <string>

namespace {

std::string describe_error(const ob::Error& error) {
    return std::string(error.what()) + " [function=" + error.getFunction() + ", args=" + error.getArgs() + "]";
}

std::shared_ptr<ob::StreamProfile> first_profile(const std::shared_ptr<ob::Pipeline>& pipeline,
                                                  const OBSensorType sensor,
                                                  const OBFormat preferred_format) {
    const auto profiles = pipeline->getStreamProfileList(sensor);
    std::cout << "profiles for " << ob::TypeHelper::convertOBSensorTypeToString(sensor) << ": " << profiles->getCount() << '\n';
    for(std::uint32_t index = 0; index < profiles->getCount(); ++index) {
        const auto profile = profiles->getProfile(index);
        const auto video = profile->as<ob::VideoStreamProfile>();
        std::cout << "  [" << index << "] " << video->getWidth() << 'x' << video->getHeight() << " @ " << video->getFps()
                  << " " << ob::TypeHelper::convertOBFormatTypeToString(video->getFormat()) << '\n';
        if(profile->getFormat() == preferred_format) {
            return profile;
        }
    }
    return profiles->getCount() == 0 ? nullptr : profiles->getProfile(0);
}

bool try_start(const char* label,
               const std::shared_ptr<ob::StreamProfile>& depth_profile,
               const std::shared_ptr<ob::StreamProfile>& color_profile) {
    try {
        auto pipeline = std::make_shared<ob::Pipeline>();
        auto config = std::make_shared<ob::Config>();
        if(depth_profile) {
            config->enableStream(depth_profile);
        }
        if(color_profile) {
            config->enableStream(color_profile);
        }
        config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
        pipeline->start(config);
        const auto frames = pipeline->waitForFrameset(2000);
        const bool depth_received = frames && frames->getDepthFrame();
        const bool color_received = frames && frames->getColorFrame();
        pipeline->stop();
        std::cout << label << ": started; depth=" << depth_received << ", color=" << color_received << '\n';
        return true;
    }
    catch(const ob::Error& error) {
        std::cout << label << ": FAILED: " << describe_error(error) << '\n';
        return false;
    }
}

}  // namespace

int main() {
    try {
        ob::Context context;
        const auto devices = context.queryDeviceList();
        std::cout << "devices: " << devices->getCount() << '\n';
        if(devices->getCount() == 0) {
            return 2;
        }

        const auto device = devices->getDevice(0);
        const auto info = device->getDeviceInfo();
        std::cout << "device: " << info->getName() << " | PID: 0x" << std::hex << info->getPid() << std::dec
                  << " | serial: " << info->getSerialNumber() << '\n';
        const auto sensors = device->getSensorList();
        std::cout << "sensors: " << sensors->getCount() << '\n';
        for(std::uint32_t index = 0; index < sensors->getCount(); ++index) {
            std::cout << "  [" << index << "] "
                      << ob::TypeHelper::convertOBSensorTypeToString(sensors->getSensorType(index)) << '\n';
        }

        auto profile_pipeline = std::make_shared<ob::Pipeline>();
        const auto depth_profile = first_profile(profile_pipeline, OB_SENSOR_DEPTH, OB_FORMAT_Y16);
        const auto color_profile = first_profile(profile_pipeline, OB_SENSOR_COLOR, OB_FORMAT_RGB);

        const bool depth_ok = try_start("depth-only", depth_profile, nullptr);
        const bool color_ok = try_start("color-only", nullptr, color_profile);
        const bool rgbd_ok = try_start("rgb-d", depth_profile, color_profile);
        return depth_ok && color_ok && rgbd_ok ? 0 : 1;
    }
    catch(const ob::Error& error) {
        std::cerr << "probe FAILED: " << describe_error(error) << '\n';
        return 1;
    }
}
