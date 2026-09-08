#include "aerial_touch/touch_state_machine.hpp"

#include <cmath>

namespace aerial_touch {
namespace {

// How far the finger may drift back up before the approach counts as abandoned. Kept tight: a
// spurious cancel above the touch zone costs nothing, because the next still-approaching frame
// re-latches immediately, whereas a latch that outlives its approach fires a key the user never
// pressed.
constexpr float kApproachRetreatToleranceMm = 3.0F;

}  // namespace

TouchStateMachine::TouchStateMachine(const TouchConfig config) : config_(config) {}

void TouchStateMachine::set_config(const TouchConfig config) {
    config_ = config;
    // armed_ deliberately survives a config change, but the latch and the dwell anchor are
    // measurements taken against the *old* thresholds. Carrying them across would let a fast
    // approach measured under one touch_threshold_mm fire the moment a wider one is applied --
    // i.e. dragging a slider in the settings window would emit a keystroke.
    reset_trigger_state();
}

void TouchStateMachine::reset_trigger_state() {
    approach_latched_ = false;
    approach_minimum_mm_.reset();
    dwell_since_ms_.reset();
    dwell_key_.reset();
}

std::optional<PressEvent> TouchStateMachine::update(const TouchSample& sample) {
    const auto previous_timestamp_ms = last_tracking_timestamp_ms_;
    const auto previous_distance_mm = previous_distance_mm_;
    if(last_tracking_timestamp_ms_.has_value()
       && sample.timestamp_ms - *last_tracking_timestamp_ms_ > config_.tracking_timeout_ms) {
        armed_ = false;
        previous_distance_mm_.reset();
        reset_trigger_state();
    }
    if(sample.signed_distance_mm >= config_.release_threshold_mm) {
        // Retreating past the release threshold starts a brand new approach, so any evidence
        // gathered for the previous one is discarded.
        armed_ = true;
        reset_trigger_state();
    }

    float approach_velocity = 0.0F;
    if(previous_distance_mm_.has_value() && previous_timestamp_ms.has_value()) {
        const float elapsed_seconds = static_cast<float>(sample.timestamp_ms - *previous_timestamp_ms) / 1000.0F;
        if(elapsed_seconds > 0.0F) {
            approach_velocity = (*previous_distance_mm_ - sample.signed_distance_mm) / elapsed_seconds;
        }
    }

    const bool approaching = previous_distance_mm.has_value() && sample.signed_distance_mm < *previous_distance_mm
                             && approach_velocity >= config_.min_approach_velocity_mm_s;
    if(approaching) {
        approach_latched_ = true;
    }
    if(approach_latched_) {
        // Track how close this approach has got. Backing away by more than the tolerance means the
        // approach is over, so its speed no longer authorises a press: a later slow crawl back down
        // must be judged on its own merits, which is exactly what min_approach_velocity_mm_s is for.
        if(!approach_minimum_mm_.has_value() || sample.signed_distance_mm < *approach_minimum_mm_) {
            approach_minimum_mm_ = sample.signed_distance_mm;
        }
        else if(sample.signed_distance_mm > *approach_minimum_mm_ + kApproachRetreatToleranceMm) {
            approach_latched_ = false;
            approach_minimum_mm_.reset();
        }
    }
    previous_distance_mm_ = sample.signed_distance_mm;
    last_tracking_timestamp_ms_ = sample.timestamp_ms;

    const bool inside_touch_zone =
        sample.signed_distance_mm <= config_.touch_threshold_mm && sample.key.has_value();
    if(inside_touch_zone) {
        if(!dwell_since_ms_.has_value() || dwell_key_ != sample.key) {
            // Sliding onto a different key restarts the dwell so a drag does not fire a chord.
            dwell_since_ms_ = sample.timestamp_ms;
            dwell_key_ = sample.key;
        }
    }
    else {
        dwell_since_ms_.reset();
        dwell_key_.reset();
    }

    const bool dwell_satisfied = config_.dwell_ms > 0 && dwell_since_ms_.has_value()
                                 && sample.timestamp_ms - *dwell_since_ms_ >= config_.dwell_ms;

    if(armed_ && inside_touch_zone && (approach_latched_ || dwell_satisfied)) {
        const PressTrigger trigger = approach_latched_ ? PressTrigger::Approach : PressTrigger::Dwell;
        armed_ = false;
        reset_trigger_state();
        return PressEvent{ *sample.key, sample.timestamp_ms, sample.fingertip_xyz_mm, sample.plane_uv_mm, trigger };
    }

    // Reaching the touch zone spends the approach. If it did not fire here -- no key under the
    // fingertip, or not armed -- then the finger is already resting at the surface, and a later
    // sideways slide onto a key must not cash in that old descent. Only a dwell (or a fresh
    // approach after retreating past the release threshold) may fire from here on.
    if(sample.signed_distance_mm <= config_.touch_threshold_mm) {
        approach_latched_ = false;
        approach_minimum_mm_.reset();
    }
    return std::nullopt;
}

void TouchStateMachine::mark_tracking_lost(const std::int64_t timestamp_ms) {
    if(last_tracking_timestamp_ms_.has_value()
       && timestamp_ms - *last_tracking_timestamp_ms_ > config_.tracking_timeout_ms) {
        armed_ = false;
    }
    previous_distance_mm_.reset();
    // Losing the finger invalidates both the velocity evidence and the dwell: neither can be
    // trusted across a gap where we did not see where the finger went.
    reset_trigger_state();
}

bool TouchStateMachine::armed() const {
    return armed_;
}

bool TouchStateMachine::approach_latched() const {
    return approach_latched_;
}

std::optional<std::int64_t> TouchStateMachine::dwell_elapsed_ms(const std::int64_t timestamp_ms) const {
    if(!dwell_since_ms_.has_value()) {
        return std::nullopt;
    }
    const std::int64_t elapsed = timestamp_ms - *dwell_since_ms_;
    return elapsed < 0 ? std::optional<std::int64_t>{ 0 } : std::optional<std::int64_t>{ elapsed };
}

}  // namespace aerial_touch
