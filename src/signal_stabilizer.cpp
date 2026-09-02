#include "aerial_touch/signal_stabilizer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace aerial_touch {
namespace {

constexpr float kPi = 3.14159265358979323846F;

float smoothing_alpha(const float cutoff_hz, const float elapsed_seconds) {
    const float time_constant = 1.0F / (2.0F * kPi * cutoff_hz);
    return 1.0F / (1.0F + time_constant / elapsed_seconds);
}

float low_pass(const float value, const float previous, const float alpha) {
    return alpha * value + (1.0F - alpha) * previous;
}

float median(std::vector<float> values) {
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
    std::nth_element(values.begin(), middle, values.end());
    if(values.size() % 2U == 1U) {
        return *middle;
    }
    const float upper = *middle;
    return (*std::max_element(values.begin(), middle) + upper) / 2.0F;
}

}  // namespace

OneEuroFilter2D::OneEuroFilter2D(const OneEuroConfig config) : config_(config) {
    if(!std::isfinite(config_.min_cutoff_hz) || config_.min_cutoff_hz <= 0.0F
       || !std::isfinite(config_.beta) || config_.beta < 0.0F
       || !std::isfinite(config_.derivative_cutoff_hz) || config_.derivative_cutoff_hz <= 0.0F) {
        throw std::invalid_argument(u8"One Euro 濾波器設定無效");
    }
}

Vec2 OneEuroFilter2D::update(const Vec2 value, const std::int64_t timestamp_ms) {
    if(!previous_raw_.has_value() || !previous_filtered_.has_value() || !previous_timestamp_ms_.has_value()
       || timestamp_ms <= *previous_timestamp_ms_) {
        previous_raw_ = value;
        previous_filtered_ = value;
        filtered_derivative_ = {};
        previous_timestamp_ms_ = timestamp_ms;
        return value;
    }

    const float elapsed_seconds = static_cast<float>(timestamp_ms - *previous_timestamp_ms_) / 1000.0F;
    const float derivative_alpha = smoothing_alpha(config_.derivative_cutoff_hz, elapsed_seconds);
    const Vec2 derivative{ (value.x - previous_raw_->x) / elapsed_seconds,
                           (value.y - previous_raw_->y) / elapsed_seconds };
    filtered_derivative_.x = low_pass(derivative.x, filtered_derivative_.x, derivative_alpha);
    filtered_derivative_.y = low_pass(derivative.y, filtered_derivative_.y, derivative_alpha);

    const float cutoff_x = config_.min_cutoff_hz + config_.beta * std::fabs(filtered_derivative_.x);
    const float cutoff_y = config_.min_cutoff_hz + config_.beta * std::fabs(filtered_derivative_.y);
    Vec2 filtered{
        low_pass(value.x, previous_filtered_->x, smoothing_alpha(cutoff_x, elapsed_seconds)),
        low_pass(value.y, previous_filtered_->y, smoothing_alpha(cutoff_y, elapsed_seconds)),
    };

    previous_raw_ = value;
    previous_filtered_ = filtered;
    previous_timestamp_ms_ = timestamp_ms;
    return filtered;
}

void OneEuroFilter2D::reset() {
    previous_raw_.reset();
    previous_filtered_.reset();
    previous_timestamp_ms_.reset();
    filtered_derivative_ = {};
}

HandSignalStabilizer::HandSignalStabilizer(const HandSignalConfig config)
    : config_(config), filter_(config.filter) {
    if(config_.display_hold_ms < 0) {
        throw std::invalid_argument(u8"指尖保留顯示時間無效");
    }
}

std::optional<StabilizedFingertip> HandSignalStabilizer::update(const std::optional<Vec2> observation,
                                                                const std::int64_t timestamp_ms) {
    if(observation.has_value() && std::isfinite(observation->x) && std::isfinite(observation->y)) {
        StabilizedFingertip value{ *observation, filter_.update(*observation, timestamp_ms), true };
        last_value_ = value;
        last_observation_timestamp_ms_ = timestamp_ms;
        return value;
    }
    if(last_value_.has_value() && last_observation_timestamp_ms_.has_value()
       && timestamp_ms >= *last_observation_timestamp_ms_
       && timestamp_ms - *last_observation_timestamp_ms_ <= config_.display_hold_ms) {
        auto held = *last_value_;
        held.confirmed_this_frame = false;
        return held;
    }
    reset();
    return std::nullopt;
}

void HandSignalStabilizer::reset() {
    filter_.reset();
    last_value_.reset();
    last_observation_timestamp_ms_.reset();
}

DepthSignalStabilizer::DepthSignalStabilizer(const DepthSignalConfig config) : config_(config) {
    if(config_.median_window_size == 0U || config_.invalid_reset_frames == 0U
       || !std::isfinite(config_.max_jump_mm) || config_.max_jump_mm <= 0.0F) {
        throw std::invalid_argument(u8"深度訊號濾波器設定無效");
    }
}

std::optional<float> DepthSignalStabilizer::update(const std::optional<float> depth_mm) {
    if(!depth_mm.has_value() || !std::isfinite(*depth_mm) || *depth_mm <= 0.0F) {
        ++invalid_frames_;
        if(invalid_frames_ >= config_.invalid_reset_frames) {
            reset();
        }
        return std::nullopt;
    }

    invalid_frames_ = 0U;
    if(!history_.empty()) {
        const float center = median({ history_.begin(), history_.end() });
        if(std::fabs(*depth_mm - center) > config_.max_jump_mm) {
            ++rejected_jump_frames_;
            if(rejected_jump_frames_ < config_.invalid_reset_frames) {
                return std::nullopt;
            }
            history_.clear();
        }
    }

    rejected_jump_frames_ = 0U;
    history_.push_back(*depth_mm);
    while(history_.size() > config_.median_window_size) {
        history_.pop_front();
    }
    return median({ history_.begin(), history_.end() });
}

void DepthSignalStabilizer::reset() {
    history_.clear();
    invalid_frames_ = 0U;
    rejected_jump_frames_ = 0U;
}

DepthFreshnessGate::DepthFreshnessGate(const std::size_t invalid_reset_frames)
    : invalid_reset_frames_(invalid_reset_frames) {
    if(invalid_reset_frames_ == 0U) {
        throw std::invalid_argument(u8"原始深度失效門檻不可為零");
    }
}

bool DepthFreshnessGate::update(const std::optional<float> raw_depth_mm) {
    if(raw_depth_mm.has_value() && std::isfinite(*raw_depth_mm) && *raw_depth_mm > 0.0F) {
        reset();
        return true;
    }
    ++consecutive_invalid_frames_;
    if(consecutive_invalid_frames_ >= invalid_reset_frames_) {
        stale_ = true;
    }
    return !stale_;
}

void DepthFreshnessGate::reset() {
    consecutive_invalid_frames_ = 0U;
    stale_ = false;
}

}  // namespace aerial_touch
