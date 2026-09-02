#pragma once

#include "aerial_touch/app_config.hpp"

#include <filesystem>
#include <functional>
#include <optional>
#include <string>

namespace aerial_touch {

struct SettingsRect {
    int left{};
    int top{};
    int right{};
    int bottom{};
};

struct SettingsLayout {
    SettingsRect touch_group;
    SettingsRect depth_group;
    SettingsRect preview_group;
    SettingsRect keypad_group;
    SettingsRect preview_rect;
    int status_y{};
    int path_y{};
    int buttons_y{};
};

SettingsLayout calculate_settings_layout(int client_width, int client_height);

enum class PreviewZone {
    Unknown,
    Touch,
    Hold,
    Release,
};

PreviewZone classify_preview_zone(std::optional<float> distance_mm,
                                  float touch_threshold_mm,
                                  float release_threshold_mm);

struct SettingsPreview {
    std::optional<float> distance_mm;
    bool tracking_detected{ false };
    std::optional<std::string> key;
    bool armed{ false };
};

class SettingsWindow {
public:
    using ApplyCallback = std::function<bool(const AppConfig&, std::string&)>;

    SettingsWindow();
    ~SettingsWindow();

    SettingsWindow(const SettingsWindow&) = delete;
    SettingsWindow& operator=(const SettingsWindow&) = delete;

    bool create(const AppConfig& config, CameraCapabilities camera_capabilities,
                std::filesystem::path config_path, ApplyCallback apply_callback);
    void show();
    void hide();
    bool visible() const;
    void process_messages();
    void update_preview(SettingsPreview preview);

private:
#ifdef _WIN32
    struct Impl;
    Impl* impl_{};
#endif
};

}  // namespace aerial_touch
