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
    // Hold inside the touch threshold for this long and the key fires even if the approach was
    // too slow to satisfy min_approach_velocity_mm_s. 0 disables the dwell path entirely.
    std::int64_t dwell_ms{ 350 };
};

struct TouchSample {
    std::int64_t timestamp_ms{};
    float signed_distance_mm{};
    std::optional<std::string> key;
    Vec3 fingertip_xyz_mm{};
    Vec2 plane_uv_mm{};
};

enum class PressTrigger {
    Approach,
    Dwell,
};

struct PressEvent {
    std::string key;
    std::int64_t timestamp_ms{};
    Vec3 fingertip_xyz_mm{};
    Vec2 plane_uv_mm{};
    PressTrigger trigger{ PressTrigger::Approach };
};

class TouchStateMachine {
public:
    explicit TouchStateMachine(TouchConfig config);

    void set_config(TouchConfig config);
    std::optional<PressEvent> update(const TouchSample& sample);
    void mark_tracking_lost(std::int64_t timestamp_ms);
    bool armed() const;

    // True once the approach has been fast enough to satisfy the velocity gate for this press.
    // The gate is latched rather than tested on the crossing frame alone: people decelerate as
    // they near a target (Fitts' law), so the instantaneous speed at the moment the finger
    // crosses touch_threshold_mm is routinely well below the speed of the approach itself.
    bool approach_latched() const;

    // Milliseconds the finger has been resting inside the touch threshold on one key, if any.
    std::optional<std::int64_t> dwell_elapsed_ms(std::int64_t timestamp_ms) const;

private:
    void reset_trigger_state();

    TouchConfig config_;
    bool armed_{ false };
    bool approach_latched_{ false };
    std::optional<float> approach_minimum_mm_;
    std::optional<float> previous_distance_mm_;
    std::optional<std::int64_t> last_tracking_timestamp_ms_;
    std::optional<std::int64_t> dwell_since_ms_;
    std::optional<std::string> dwell_key_;
};

}  // namespace aerial_touch
