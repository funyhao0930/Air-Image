#include "aerial_touch/touch_state_machine.hpp"

#include <cmath>

namespace aerial_touch {

TouchStateMachine::TouchStateMachine(const TouchConfig config) : config_(config) {}

void TouchStateMachine::set_config(const TouchConfig config) {
    config_ = config;
}

std::optional<PressEvent> TouchStateMachine::update(const TouchSample& sample) {
    const auto previous_timestamp_ms = last_tracking_timestamp_ms_;
    if(last_tracking_timestamp_ms_.has_value()
       && sample.timestamp_ms - *last_tracking_timestamp_ms_ > config_.tracking_timeout_ms) {
        armed_ = false;
        previous_distance_mm_.reset();
    }
    if(sample.signed_distance_mm >= config_.release_threshold_mm) {
        armed_ = true;
    }

    float approach_velocity = 0.0F;
    if(previous_distance_mm_.has_value() && previous_timestamp_ms.has_value()) {
        const float elapsed_seconds = static_cast<float>(sample.timestamp_ms - *previous_timestamp_ms) / 1000.0F;
        if(elapsed_seconds > 0.0F) {
            approach_velocity = (*previous_distance_mm_ - sample.signed_distance_mm) / elapsed_seconds;
        }
    }

    const bool approaching = previous_distance_mm_.has_value() && sample.signed_distance_mm < *previous_distance_mm_
                             && approach_velocity >= config_.min_approach_velocity_mm_s;
    previous_distance_mm_ = sample.signed_distance_mm;
    last_tracking_timestamp_ms_ = sample.timestamp_ms;

    if(armed_ && approaching && sample.signed_distance_mm <= config_.touch_threshold_mm && sample.key.has_value()) {
        armed_ = false;
        return PressEvent{ *sample.key, sample.timestamp_ms, sample.fingertip_xyz_mm, sample.plane_uv_mm };
    }
    return std::nullopt;
}

void TouchStateMachine::mark_tracking_lost(const std::int64_t timestamp_ms) {
    if(last_tracking_timestamp_ms_.has_value() && timestamp_ms - *last_tracking_timestamp_ms_ > config_.tracking_timeout_ms) {
        armed_ = false;
        previous_distance_mm_.reset();
    }
}

bool TouchStateMachine::armed() const {
    return armed_;
}

}  // namespace aerial_touch
