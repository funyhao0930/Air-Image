#include <libobsensor/ObSensor.hpp>

#include <iostream>
#include <memory>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

std::string describe_error(const ob::Error& error) {
    return std::string(error.what()) + " [函式=" + error.getFunction() + ", 參數=" + error.getArgs() + "]";
}

void print_profiles(const std::shared_ptr<ob::Pipeline>& pipeline, const OBSensorType sensor) {
    const auto profiles = pipeline->getStreamProfileList(sensor);
    std::cout << "可用串流設定（" << ob::TypeHelper::convertOBSensorTypeToString(sensor) << "）：" << profiles->getCount() << '\n';
    for(std::uint32_t index = 0; index < profiles->getCount(); ++index) {
        const auto profile = profiles->getProfile(index);
        const auto video = profile->as<ob::VideoStreamProfile>();
        std::cout << "  [" << index << "] " << video->getWidth() << 'x' << video->getHeight() << " @ " << video->getFps()
                  << " " << ob::TypeHelper::convertOBFormatTypeToString(video->getFormat()) << '\n';
    }
}

std::shared_ptr<ob::Config> hardware_align_config(const std::shared_ptr<ob::Pipeline>& pipeline) {
    const auto color_profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    for(std::uint32_t color_index = 0; color_index < color_profiles->getCount(); ++color_index) {
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
            return nullptr;
        }

        for(std::uint32_t depth_index = 0; depth_index < depth_profiles->getCount(); ++depth_index) {
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

}  // namespace

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    try {
        ob::Context context;
        const auto devices = context.queryDeviceList();
        std::cout << "裝置數量：" << devices->getCount() << '\n';
        if(devices->getCount() == 0) {
            return 2;
        }

        const auto device = devices->getDevice(0);
        const auto info = device->getDeviceInfo();
        std::cout << "裝置：" << info->getName() << " | PID：0x" << std::hex << info->getPid() << std::dec
                  << " | 序號：" << info->getSerialNumber() << '\n';
        const auto sensors = device->getSensorList();
        std::cout << "感測器數量：" << sensors->getCount() << '\n';
        for(std::uint32_t index = 0; index < sensors->getCount(); ++index) {
            std::cout << "  [" << index << "] "
                      << ob::TypeHelper::convertOBSensorTypeToString(sensors->getSensorType(index)) << '\n';
        }

        auto pipeline = std::make_shared<ob::Pipeline>();
        pipeline->enableFrameSync();
        print_profiles(pipeline, OB_SENSOR_DEPTH);
        print_profiles(pipeline, OB_SENSOR_COLOR);

        const auto config = hardware_align_config(pipeline);
        if(!config) {
            std::cerr << "探測失敗：此相機找不到可用的 RGB-D 硬體對齊設定\n";
            return 1;
        }

        pipeline->start(config);
        const auto frames = pipeline->waitForFrameset(5000);
        const bool depth_received = frames && frames->getDepthFrame();
        const bool color_received = frames && frames->getColorFrame();
        pipeline->stop();
        std::cout << "RGB-D 硬體對齊：啟動" << (depth_received && color_received ? "成功" : "失敗")
                  << "；深度=" << depth_received << "，彩色=" << color_received << '\n';
        return depth_received && color_received ? 0 : 1;
    }
    catch(const ob::Error& error) {
        std::cerr << "探測失敗：" << describe_error(error) << '\n';
        return 1;
    }
}
