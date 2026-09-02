#pragma once

#include "aerial_touch/types.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace aerial_touch {

struct TouchConfig {
    float touch_threshold_mm{ 10.0F };
    float release_threshold_mm{ 20.0F };
    float min_approach_velocity_mm_s{ 40.0F };
    std::int64_t tracking_timeout_ms{ 300 };
};

struct TouchSample {
    std::int64_t timestamp_ms{};
    float signed_distance_mm{};
    std::optional<std::string> key;
    Vec3 fingertip_xyz_mm{};
    Vec2 plane_uv_mm{};
};

struct PressEvent {
    std::string key;
    std::int64_t timestamp_ms{};
    Vec3 fingertip_xyz_mm{};
    Vec2 plane_uv_mm{};
};

class TouchStateMachine {
public:
    explicit TouchStateMachine(TouchConfig config);

    void set_config(TouchConfig config);
    std::optional<PressEvent> update(const TouchSample& sample);
    void mark_tracking_lost(std::int64_t timestamp_ms);
    bool armed() const;

private:
    TouchConfig config_;
    bool armed_{ false };
    std::optional<float> previous_distance_mm_;
    std::optional<std::int64_t> last_tracking_timestamp_ms_;
};

}  // namespace aerial_touch
