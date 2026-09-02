#include "aerial_touch/calibration_sampler.hpp"
#include "aerial_touch/keypad.hpp"
#include "aerial_touch/signal_stabilizer.hpp"

#include <cmath>
#include <optional>

namespace {

bool near(const float actual, const float expected, const float tolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

bool one_euro_filter_converges_for_stationary_input() {
    aerial_touch::OneEuroFilter2D filter({ 1.0F, 0.12F, 1.0F });
    aerial_touch::Vec2 filtered{};
    for(int frame = 0; frame < 90; ++frame) {
        const float noise = frame % 2 == 0 ? 2.0F : -2.0F;
        filtered = filter.update({ 100.0F + noise, 200.0F - noise }, frame * 33);
    }
    return near(filtered.x, 100.0F, 1.0F) && near(filtered.y, 200.0F, 1.0F);
}

bool one_euro_filter_reduces_lag_during_fast_motion() {
    aerial_touch::OneEuroFilter2D filter({ 1.0F, 0.12F, 1.0F });
    static_cast<void>(filter.update({ 0.0F, 0.0F }, 0));
    const auto filtered = filter.update({ 200.0F, 0.0F }, 33);
    return filtered.x >= 150.0F && filtered.x <= 200.0F;
}

bool hand_stabilizer_holds_display_without_confirming_events() {
    aerial_touch::HandSignalStabilizer stabilizer({ { 1.0F, 0.12F, 1.0F }, 100 });
    const auto observed = stabilizer.update(aerial_touch::Vec2{ 10.0F, 20.0F }, 0);
    const auto held = stabilizer.update(std::nullopt, 50);
    const auto expired = stabilizer.update(std::nullopt, 101);
    return observed.has_value() && observed->confirmed_this_frame
           && held.has_value() && !held->confirmed_this_frame
           && near(held->filtered.x, observed->filtered.x, 0.001F)
           && !expired.has_value();
}

bool depth_filter_rejects_single_frame_jump() {
    aerial_touch::DepthSignalStabilizer filter({ 5, 80.0F, 3 });
    static_cast<void>(filter.update(1000.0F));
    const auto stable = filter.update(1002.0F);
    const auto outlier = filter.update(1400.0F);
    const auto recovered = filter.update(1004.0F);
    return stable.has_value() && !outlier.has_value() && recovered.has_value()
           && *recovered < 1050.0F;
}

bool invalid_depth_clears_history() {
    aerial_touch::DepthSignalStabilizer filter({ 5, 80.0F, 3 });
    static_cast<void>(filter.update(1000.0F));
    static_cast<void>(filter.update(std::nullopt));
    static_cast<void>(filter.update(std::nullopt));
    const auto cleared = filter.update(std::nullopt);
    const auto fresh = filter.update(700.0F);
    return !cleared.has_value() && fresh.has_value() && near(*fresh, 700.0F, 0.001F);
}

bool repeated_depth_jump_reseeds_at_new_distance() {
    aerial_touch::DepthSignalStabilizer filter({ 5, 80.0F, 3 });
    static_cast<void>(filter.update(1000.0F));
    const auto rejected1 = filter.update(1400.0F);
    const auto rejected2 = filter.update(1400.0F);
    const auto reseeded = filter.update(1400.0F);
    return !rejected1.has_value() && !rejected2.has_value()
           && reseeded.has_value() && near(*reseeded, 1400.0F, 0.001F);
}

bool stale_temporal_depth_is_blocked_until_raw_depth_returns() {
    aerial_touch::DepthFreshnessGate gate(3);
    return gate.update(1000.0F)
           && gate.update(std::nullopt)
           && gate.update(std::nullopt)
           && !gate.update(std::nullopt)
           && !gate.update(std::nullopt)
           && gate.update(700.0F);
}

bool calibration_uses_robust_median_with_outlier() {
    aerial_touch::CalibrationSampleCollector collector({ 15, 3.5F, 2.0F });
    for(int sample = 0; sample < 16; ++sample) {
        const float offset = static_cast<float>((sample % 5) - 2);
        collector.add({ 100.0F + offset, 200.0F - offset, 900.0F + offset });
    }
    collector.add({ 800.0F, -500.0F, 1800.0F });

    const auto result = collector.result();
    return result.has_value() && result->accepted_samples >= 15U
           && near(result->point.x, 100.0F, 1.0F)
           && near(result->point.y, 200.0F, 1.0F)
           && near(result->point.z, 900.0F, 1.0F)
           && result->spread.x <= 4.0F && result->spread.y <= 4.0F && result->spread.z <= 4.0F;
}

bool calibration_requires_enough_samples_and_caps_collection() {
    aerial_touch::CalibrationSampleCollector collector({ 20, 3.5F, 2.0F });
    for(int sample = 0; sample < 14; ++sample) {
        collector.add({ 100.0F, 200.0F, 900.0F });
    }
    if(collector.result().has_value()) {
        return false;
    }
    for(int sample = 14; sample < 20; ++sample) {
        collector.add({ 100.0F, 200.0F, 900.0F });
    }
    return collector.result().has_value()
           && !collector.add({ 100.0F, 200.0F, 900.0F })
           && collector.sample_count() == 20U;
}

bool keypad_boundary_hysteresis_ignores_small_jitter() {
    const aerial_touch::Keypad keypad({ 30.0F, 30.0F, 5.0F, 5.0F });
    const auto held = keypad.key_at({ 30.5F, 15.0F }, std::optional<std::string>{ "1" }, 2.0F);
    const auto released = keypad.key_at({ 32.5F, 15.0F }, std::optional<std::string>{ "1" }, 2.0F);
    return held.value_or("?") == "1" && !released.has_value();
}

}  // namespace

bool run_signal_stability_tests() {
    return one_euro_filter_converges_for_stationary_input()
           && one_euro_filter_reduces_lag_during_fast_motion()
           && hand_stabilizer_holds_display_without_confirming_events()
           && depth_filter_rejects_single_frame_jump()
           && invalid_depth_clears_history()
           && repeated_depth_jump_reseeds_at_new_distance()
           && stale_temporal_depth_is_blocked_until_raw_depth_returns()
           && calibration_uses_robust_median_with_outlier()
           && calibration_requires_enough_samples_and_caps_collection()
           && keypad_boundary_hysteresis_ignores_small_jitter();
}
