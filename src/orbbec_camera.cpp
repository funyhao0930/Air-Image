#include "aerial_touch/orbbec_camera.hpp"
#include "aerial_touch/alignment_mode.hpp"

#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Utils.hpp>

#include <algorithm>
#include <array>
#include <cstring>
#include <exception>
#include <sstream>
#include <vector>

namespace aerial_touch {
namespace {

struct ProfilePair {
    std::shared_ptr<ob::StreamProfile> color;
    std::shared_ptr<ob::StreamProfile> depth;
};

template<typename T>
void append_unique(std::vector<T>& values, T value) {
    if(std::find(values.begin(), values.end(), value) == values.end()) {
        values.push_back(std::move(value));
    }
}

std::optional<ProfilePair> hardware_profile_pair(const std::shared_ptr<ob::Pipeline>& pipeline,
                                                 const int preferred_fps,
                                                 std::vector<int>* supported_fps = nullptr) {
    const auto color_profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    std::vector<ProfilePair> pairs;
    std::vector<CameraProfileOption> options;
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
            continue;
        }
        for(std::uint32_t depth_index = 0; depth_index < depth_profiles->getCount(); ++depth_index) {
            const auto depth_profile = depth_profiles->getProfile(depth_index);
            const auto depth_video = depth_profile->as<ob::VideoStreamProfile>();
            const auto index = pairs.size();
            pairs.push_back({ color_profile, depth_profile });
            options.push_back({ index, static_cast<int>(color_video->getFps()),
                                static_cast<int>(depth_video->getFps()), true });
        }
    }
    if(supported_fps != nullptr) {
        *supported_fps = supported_camera_fps(options);
    }
    const auto selected = select_camera_profile_option(preferred_fps, options);
    return selected.has_value() ? std::optional<ProfilePair>{ pairs[*selected] } : std::nullopt;
}

std::optional<ProfilePair> software_profile_pair(const std::shared_ptr<ob::Pipeline>& pipeline,
                                                 const int preferred_fps,
                                                 std::vector<int>* supported_fps = nullptr) {
    const auto color_profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    const auto depth_profiles = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
    std::vector<ProfilePair> pairs;
    std::vector<CameraProfileOption> options;
    for(std::uint32_t color_index = 0; color_index < color_profiles->getCount(); ++color_index) {
        const auto color_profile = color_profiles->getProfile(color_index);
        const auto color_video = color_profile->as<ob::VideoStreamProfile>();
        for(std::uint32_t depth_index = 0; depth_index < depth_profiles->getCount(); ++depth_index) {
            const auto depth_profile = depth_profiles->getProfile(depth_index);
            const auto depth_video = depth_profile->as<ob::VideoStreamProfile>();
            const auto index = pairs.size();
            pairs.push_back({ color_profile, depth_profile });
            options.push_back({ index, static_cast<int>(color_video->getFps()),
                                static_cast<int>(depth_video->getFps()),
                                color_video->getFormat() == OB_FORMAT_RGB });
        }
    }
    if(supported_fps != nullptr) {
        *supported_fps = supported_camera_fps(options);
    }
    const auto selected = select_camera_profile_option(preferred_fps, options);
    return selected.has_value() ? std::optional<ProfilePair>{ pairs[*selected] } : std::nullopt;
}

void record_profile(const ProfilePair& pair, CameraRuntimeInfo& info) {
    const auto color = pair.color->as<ob::VideoStreamProfile>();
    const auto depth = pair.depth->as<ob::VideoStreamProfile>();
    info.color_width = color->getWidth();
    info.color_height = color->getHeight();
    info.depth_width = depth->getWidth();
    info.depth_height = depth->getHeight();
    info.fps = color->getFps();
}

std::shared_ptr<ob::Config> stream_config(const ProfilePair& pair, const bool hardware_alignment) {
    auto config = std::make_shared<ob::Config>();
    config->enableStream(pair.color);
    config->enableStream(pair.depth);
    if(hardware_alignment) {
        config->setAlignMode(ALIGN_D2C_HW_MODE);
    }
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    return config;
}

std::string describe_error(const ob::Error& error) {
    return std::string(error.what()) + u8" [函式=" + error.getFunction() + u8", 參數=" + error.getArgs() + "]";
}

std::optional<OBDepthPrecisionLevel> parse_depth_precision(const std::string& value) {
    static const std::array<std::pair<const char*, OBDepthPrecisionLevel>, 7> values{ {
        { "1mm", OB_PRECISION_1MM }, { "0.8mm", OB_PRECISION_0MM8 }, { "0.4mm", OB_PRECISION_0MM4 },
        { "0.1mm", OB_PRECISION_0MM1 }, { "0.2mm", OB_PRECISION_0MM2 }, { "0.5mm", OB_PRECISION_0MM5 },
        { "0.05mm", OB_PRECISION_0MM05 },
    } };
    const auto found = std::find_if(values.begin(), values.end(), [&](const auto& item) {
        return value == item.first;
    });
    return found == values.end() ? std::nullopt : std::optional<OBDepthPrecisionLevel>{ found->second };
}

std::string depth_precision_name(const int value) {
    static const std::array<const char*, 7> names{ "1mm", "0.8mm", "0.4mm", "0.1mm", "0.2mm", "0.5mm", "0.05mm" };
    return value >= 0 && value < static_cast<int>(names.size()) ? names[static_cast<std::size_t>(value)] : u8"未知";
}

std::vector<int> supported_depth_precision_values(const std::shared_ptr<ob::Device>& device) {
    std::vector<int> supported;
    if(device->isPropertySupported(OB_STRUCT_DEPTH_PRECISION_SUPPORT_LIST, OB_PERMISSION_READ)) {
        std::array<std::uint16_t, OB_PRECISION_COUNT> values{};
        std::uint32_t bytes = static_cast<std::uint32_t>(sizeof(values));
        device->getStructuredData(OB_STRUCT_DEPTH_PRECISION_SUPPORT_LIST,
                                  reinterpret_cast<std::uint8_t*>(values.data()), &bytes);
        if(bytes <= sizeof(values) && bytes % sizeof(std::uint16_t) == 0U) {
            for(std::size_t index = 0; index < bytes / sizeof(std::uint16_t); ++index) {
                const int value = static_cast<int>(values[index]);
                if(value >= 0 && value < OB_PRECISION_COUNT) {
                    append_unique(supported, value);
                }
            }
        }
    }
    if(supported.empty() && device->isPropertySupported(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PERMISSION_READ)) {
        const auto range = device->getIntPropertyRange(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
        for(int value = range.min; value <= range.max; value += std::max(1, range.step)) {
            if(value >= 0 && value < OB_PRECISION_COUNT) {
                append_unique(supported, value);
            }
        }
    }
    return supported;
}

std::vector<int> supported_power_line_frequencies_hz(const std::shared_ptr<ob::Device>& device) {
    std::vector<int> supported;
    if(!device->isPropertySupported(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, OB_PERMISSION_WRITE)) {
        return supported;
    }
    const auto range = device->getIntPropertyRange(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT);
    for(int value = range.min; value <= range.max; value += std::max(1, range.step)) {
        if(value == OB_POWER_LINE_FREQ_MODE_50HZ) {
            append_unique(supported, 50);
        }
        else if(value == OB_POWER_LINE_FREQ_MODE_60HZ) {
            append_unique(supported, 60);
        }
    }
    return supported;
}

void add_warning(CameraRuntimeInfo& info, std::string warning) {
    info.warnings.push_back(std::move(warning));
}

const char* depth_filter_name(const DepthFilterKind kind) {
    switch(kind) {
    case DepthFilterKind::Temporal:
        return u8"時間濾波器";
    case DepthFilterKind::Spatial:
        return u8"空間濾波器";
    case DepthFilterKind::HoleFilling:
        return u8"孔洞填補濾波器";
    }
    return u8"未知濾波器";
}

}  // namespace

struct ActiveDepthFilter {
    DepthFilterKind kind;
    std::shared_ptr<ob::Filter> filter;
};

struct OrbbecCamera::Impl {
    std::shared_ptr<ob::Pipeline> pipeline;
    std::shared_ptr<ob::Align> software_aligner;
    std::vector<ActiveDepthFilter> depth_filters;
    bool running{ false };
    bool hardware_alignment{ false };
    std::string error;
    CameraRuntimeInfo runtime_info;
};

OrbbecCamera::OrbbecCamera() : impl_(std::make_unique<Impl>()) {}

OrbbecCamera::~OrbbecCamera() {
    stop();
}

bool OrbbecCamera::start(const CameraConfig& requested) {
    stop();
    impl_->error.clear();
    impl_->runtime_info = {};
    try {
        impl_->pipeline = std::make_shared<ob::Pipeline>();
        auto device = impl_->pipeline->getDevice();
        auto& capabilities = impl_->runtime_info.capabilities;

        try {
            const auto modes = device->getDepthWorkModeList();
            capabilities.depth_work_modes.reserve(modes->getCount());
            for(std::uint32_t index = 0; index < modes->getCount(); ++index) {
                append_unique(capabilities.depth_work_modes, std::string((*modes)[index].name));
            }
        }
        catch(const ob::Error& error) {
            add_warning(impl_->runtime_info, std::string(u8"無法查詢深度工作模式清單：") + describe_error(error));
        }

        if(!requested.depth_work_mode.empty()) {
            try {
                const auto selected_mode = select_supported_setting(requested.depth_work_mode,
                                                                    capabilities.depth_work_modes);
                if(selected_mode.has_value() && *selected_mode != device->getCurrentDepthModeName()) {
                    if(device->switchDepthWorkMode(selected_mode->c_str()) == OB_STATUS_OK) {
                        impl_->pipeline.reset();
                        impl_->pipeline = std::make_shared<ob::Pipeline>();
                        device = impl_->pipeline->getDevice();
                    }
                    else {
                        add_warning(impl_->runtime_info, u8"深度工作模式切換失敗，保留裝置目前模式");
                    }
                }
                else if(!selected_mode.has_value()) {
                    add_warning(impl_->runtime_info, std::string(u8"裝置不支援深度工作模式：")
                                                          + requested.depth_work_mode);
                }
            }
            catch(const ob::Error& error) {
                add_warning(impl_->runtime_info, std::string(u8"無法套用深度工作模式：")
                                                      + describe_error(error));
            }
        }
        try {
            impl_->runtime_info.depth_work_mode = device->getCurrentDepthModeName();
            capabilities.current_depth_work_mode = impl_->runtime_info.depth_work_mode;
            append_unique(capabilities.depth_work_modes, impl_->runtime_info.depth_work_mode);
        }
        catch(const ob::Error&) {
            add_warning(impl_->runtime_info, u8"裝置未提供目前深度工作模式");
        }

        std::vector<int> supported_precisions;
        std::optional<int> current_precision;
        try {
            supported_precisions = supported_depth_precision_values(device);
            if(device->isPropertySupported(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PERMISSION_READ)) {
                current_precision = device->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
            }
            if(!device->isPropertySupported(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PERMISSION_WRITE)) {
                supported_precisions.clear();
                if(current_precision.has_value()) {
                    supported_precisions.push_back(*current_precision);
                }
            }
            for(const int value : supported_precisions) {
                const std::string name = depth_precision_name(value);
                if(name != u8"未知") {
                    append_unique(capabilities.depth_precisions, name);
                }
            }
        }
        catch(const ob::Error& error) {
            add_warning(impl_->runtime_info, std::string(u8"無法查詢深度精度清單：") + describe_error(error));
        }

        const auto requested_precision = parse_depth_precision(requested.depth_precision);
        if(!requested_precision.has_value()) {
            add_warning(impl_->runtime_info, std::string(u8"無法識別深度精度：")
                                                  + requested.depth_precision);
        }
        else {
            try {
                const auto selected = select_supported_setting(static_cast<int>(*requested_precision), supported_precisions);
                if(selected.has_value() && current_precision.has_value() && *selected == *current_precision) {
                    // Already active; no write is needed.
                }
                else if(selected.has_value()
                        && device->isPropertySupported(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PERMISSION_WRITE)) {
                    device->setIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, *selected);
                }
                else {
                    add_warning(impl_->runtime_info, std::string(u8"裝置不支援深度精度：")
                                                          + requested.depth_precision);
                }
            }
            catch(const ob::Error& error) {
                add_warning(impl_->runtime_info, std::string(u8"無法套用深度精度：")
                                                      + describe_error(error));
            }
        }
        try {
            if(device->isPropertySupported(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PERMISSION_READ)) {
                impl_->runtime_info.depth_precision =
                    depth_precision_name(device->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT));
                capabilities.current_depth_precision = impl_->runtime_info.depth_precision;
                if(impl_->runtime_info.depth_precision != u8"未知") {
                    append_unique(capabilities.depth_precisions, impl_->runtime_info.depth_precision);
                }
            }
        }
        catch(const ob::Error&) {
            add_warning(impl_->runtime_info, u8"裝置未提供實際深度精度");
        }

        try {
            capabilities.rgb_power_line_frequencies_hz = supported_power_line_frequencies_hz(device);
            const auto selected_frequency = select_supported_setting(requested.rgb_power_line_frequency_hz,
                                                                     capabilities.rgb_power_line_frequencies_hz);
            if(selected_frequency.has_value()) {
                const int frequency = *selected_frequency == 50 ? OB_POWER_LINE_FREQ_MODE_50HZ
                                                                 : OB_POWER_LINE_FREQ_MODE_60HZ;
                device->setIntProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, frequency);
            }
            else if(capabilities.rgb_power_line_frequencies_hz.empty()) {
                add_warning(impl_->runtime_info, u8"裝置不支援設定 RGB 防閃爍頻率");
            }
            else {
                add_warning(impl_->runtime_info, u8"裝置不支援指定的 RGB 防閃爍頻率");
            }
        }
        catch(const ob::Error& error) {
            add_warning(impl_->runtime_info, std::string(u8"無法套用 RGB 防閃爍頻率：")
                                                  + describe_error(error));
        }

        impl_->pipeline->enableFrameSync();
        std::vector<int> supported_fps;
        auto pair = hardware_profile_pair(impl_->pipeline, requested.preferred_fps, &supported_fps);
        const auto alignment_mode = choose_alignment_mode(pair.has_value());
        impl_->hardware_alignment = alignment_mode == AlignmentMode::Hardware;
        impl_->runtime_info.hardware_alignment = impl_->hardware_alignment;
        if(alignment_mode == AlignmentMode::Software) {
            pair = software_profile_pair(impl_->pipeline, requested.preferred_fps, &supported_fps);
            if(!pair.has_value()) {
                throw std::runtime_error(u8"找不到具有相同 FPS 的 RGB 與深度串流設定");
            }
            impl_->software_aligner = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            impl_->software_aligner->setMatchTargetResolution(true);
        }
        record_profile(*pair, impl_->runtime_info);
        capabilities.fps_values = std::move(supported_fps);
        capabilities.current_fps = impl_->runtime_info.fps;
        append_unique(capabilities.fps_values, impl_->runtime_info.fps);
        std::sort(capabilities.fps_values.begin(), capabilities.fps_values.end());
        if(impl_->runtime_info.fps != requested.preferred_fps) {
            add_warning(impl_->runtime_info,
                        std::string(u8"找不到指定 FPS，改用 ") + std::to_string(impl_->runtime_info.fps));
        }

        std::vector<std::shared_ptr<ob::Filter>> recommended_filters;
        try {
            const auto depth_sensor = device->getSensor(OB_SENSOR_DEPTH);
            recommended_filters = depth_sensor->createRecommendedFilters();
        }
        catch(const ob::Error& error) {
            add_warning(impl_->runtime_info, std::string(u8"無法查詢裝置建議的深度濾波器：")
                                                  + describe_error(error));
        }
        bool temporal_available = false;
        bool spatial_available = false;
        bool hole_filling_available = false;
        std::shared_ptr<ob::Filter> temporal_filter;
        std::shared_ptr<ob::Filter> spatial_filter;
        std::shared_ptr<ob::Filter> hole_filling_filter;
        for(const auto& filter : recommended_filters) {
            try {
                if(!temporal_available && filter->is<ob::TemporalFilter>()) {
                    temporal_available = true;
                    if(requested.sdk_temporal_filter) {
                        const auto temporal = filter->as<ob::TemporalFilter>();
                        temporal->setDiffScale(0.1F);
                        temporal->setWeight(0.4F);
                        temporal->enable(true);
                        temporal_filter = temporal;
                    }
                }
                else if(!spatial_available && filter->is<ob::SpatialAdvancedFilter>()) {
                    spatial_available = true;
                    if(requested.sdk_spatial_filter) {
                        const auto spatial = filter->as<ob::SpatialAdvancedFilter>();
                        auto parameters = spatial->getFilterParams();
                        parameters.alpha = 0.5F;
                        parameters.disp_diff = 160U;
                        parameters.magnitude = 1U;
                        parameters.radius = 1U;
                        spatial->setFilterParams(parameters);
                        spatial->enable(true);
                        spatial_filter = spatial;
                    }
                }
                else if(!hole_filling_available && filter->is<ob::HoleFillingFilter>()) {
                    hole_filling_available = true;
                    if(requested.hole_filling_filter) {
                        filter->enable(true);
                        hole_filling_filter = filter;
                    }
                }
            }
            catch(const ob::Error& error) {
                add_warning(impl_->runtime_info, std::string(u8"深度濾波器設定失敗，已跳過：")
                                                      + describe_error(error));
            }
        }
        capabilities.temporal_filter_available = temporal_available;
        capabilities.spatial_filter_available = spatial_available;
        capabilities.hole_filling_filter_available = hole_filling_available;
        for(const auto kind : depth_filter_plan(requested, temporal_filter != nullptr, spatial_filter != nullptr,
                                                hole_filling_filter != nullptr)) {
            switch(kind) {
            case DepthFilterKind::Temporal:
                impl_->depth_filters.push_back({ kind, temporal_filter });
                break;
            case DepthFilterKind::Spatial:
                impl_->depth_filters.push_back({ kind, spatial_filter });
                break;
            case DepthFilterKind::HoleFilling:
                impl_->depth_filters.push_back({ kind, hole_filling_filter });
                break;
            }
        }
        impl_->runtime_info.temporal_filter = requested.sdk_temporal_filter && temporal_filter != nullptr;
        impl_->runtime_info.spatial_filter = requested.sdk_spatial_filter && spatial_filter != nullptr;
        impl_->runtime_info.hole_filling_filter = requested.hole_filling_filter && hole_filling_filter != nullptr;
        if(requested.sdk_temporal_filter && !temporal_available) {
            add_warning(impl_->runtime_info, u8"裝置未提供 SDK 時間濾波器，已安全停用");
        }
        if(requested.sdk_spatial_filter && !spatial_available) {
            add_warning(impl_->runtime_info, u8"裝置未提供 SDK 空間濾波器，已安全停用");
        }
        if(requested.hole_filling_filter && !hole_filling_available) {
            add_warning(impl_->runtime_info, u8"裝置未提供孔洞填補濾波器，已安全停用");
        }

        const auto config = stream_config(*pair, impl_->hardware_alignment);
        impl_->pipeline->start(config);
        impl_->running = true;
        return true;
    }
    catch(const ob::Error& error) {
        impl_->error = std::string(u8"相機啟動失敗：") + describe_error(error);
    }
    catch(const std::exception& error) {
        impl_->error = std::string(u8"相機啟動失敗：") + error.what();
    }
    impl_->pipeline.reset();
    impl_->software_aligner.reset();
    impl_->depth_filters.clear();
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
        impl_->depth_filters.clear();
    }
}

void OrbbecCamera::reset_depth_filters() {
    for(const auto& stage : impl_->depth_filters) {
        try {
            stage.filter->reset();
        }
        catch(...) {
        }
    }
}

std::optional<RgbdFrame> OrbbecCamera::capture(const std::uint32_t timeout_ms) {
    if(!impl_->running || !impl_->pipeline) {
        impl_->error = u8"Orbbec 相機尚未啟動";
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
        auto depth = frameset->getDepthFrame();
        if(!color || !depth || color->getFormat() != OB_FORMAT_RGB) {
            impl_->error = u8"影像幀缺少 RGB 或已對齊的深度資料";
            return std::nullopt;
        }
        const auto raw_depth = depth;
        std::vector<DepthFilterKind> filter_kinds;
        filter_kinds.reserve(impl_->depth_filters.size());
        for(const auto& stage : impl_->depth_filters) {
            filter_kinds.push_back(stage.kind);
        }
        const auto filtered = process_resilient_filter_chain(
            std::shared_ptr<ob::Frame>{ depth }, filter_kinds,
            [&](const DepthFilterKind kind, const std::shared_ptr<ob::Frame>& input) {
                const auto stage = std::find_if(impl_->depth_filters.begin(), impl_->depth_filters.end(),
                                                [&](const ActiveDepthFilter& candidate) {
                                                    return candidate.kind == kind;
                                                });
                return stage == impl_->depth_filters.end() ? std::shared_ptr<ob::Frame>{}
                                                           : stage->filter->process(input);
            });
        for(const auto failed_kind : filtered.failed_stages) {
            impl_->depth_filters.erase(
                std::remove_if(impl_->depth_filters.begin(), impl_->depth_filters.end(),
                               [&](const ActiveDepthFilter& stage) { return stage.kind == failed_kind; }),
                impl_->depth_filters.end());
            switch(failed_kind) {
            case DepthFilterKind::Temporal:
                impl_->runtime_info.temporal_filter = false;
                break;
            case DepthFilterKind::Spatial:
                impl_->runtime_info.spatial_filter = false;
                break;
            case DepthFilterKind::HoleFilling:
                impl_->runtime_info.hole_filling_filter = false;
                break;
            }
            add_warning(impl_->runtime_info,
                        std::string(u8"SDK ") + depth_filter_name(failed_kind)
                            + u8"執行失敗，已停用並繼續使用上一階段深度");
        }
        if(filtered.frame) {
            depth = filtered.frame->as<ob::DepthFrame>();
        }

        RgbdFrame frame;
        frame.color_width = static_cast<int>(color->getWidth());
        frame.color_height = static_cast<int>(color->getHeight());
        frame.depth_width = static_cast<int>(depth->getWidth());
        frame.depth_height = static_cast<int>(depth->getHeight());
        frame.timestamp_ms = static_cast<std::int64_t>(color->getTimeStampUs() / 1000U);
        frame.depth_unit_mm = depth->getValueScale();

        if(static_cast<int>(raw_depth->getWidth()) != frame.depth_width
           || static_cast<int>(raw_depth->getHeight()) != frame.depth_height) {
            impl_->error = u8"原始與濾波後深度影像尺寸不一致";
            return std::nullopt;
        }

        const auto rgb_bytes = static_cast<std::size_t>(frame.color_width) * static_cast<std::size_t>(frame.color_height) * 3U;
        const auto depth_values = static_cast<std::size_t>(frame.depth_width) * static_cast<std::size_t>(frame.depth_height);
        if(color->getDataSize() < rgb_bytes || depth->getDataSize() < depth_values * sizeof(std::uint16_t)
           || raw_depth->getDataSize() < depth_values * sizeof(std::uint16_t)) {
            impl_->error = u8"影像緩衝區小於串流設定所需大小";
            return std::nullopt;
        }
        frame.rgb.resize(rgb_bytes);
        frame.raw_depth.resize(depth_values);
        frame.depth.resize(depth_values);
        std::memcpy(frame.rgb.data(), color->getData(), rgb_bytes);
        std::memcpy(frame.raw_depth.data(), raw_depth->getData(), depth_values * sizeof(std::uint16_t));
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
            impl_->error = u8"RGB 與已對齊的深度影像沒有共用有效的像素座標";
            return std::nullopt;
        }
        impl_->error.clear();
        return frame;
    }
    catch(const ob::Error& error) {
        impl_->error = std::string(u8"Orbbec 影像擷取失敗：") + describe_error(error);
    }
    catch(const std::exception& error) {
        impl_->error = std::string(u8"Orbbec 影像擷取失敗：") + error.what();
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

const CameraRuntimeInfo& OrbbecCamera::runtime_info() const {
    return impl_->runtime_info;
}

const std::string& OrbbecCamera::error() const {
    return impl_->error;
}

}  // namespace aerial_touch
