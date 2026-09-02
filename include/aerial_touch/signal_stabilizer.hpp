#pragma once

#include "aerial_touch/types.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace aerial_touch {

struct OneEuroConfig {
    float min_cutoff_hz{ 1.0F };
    float beta{ 0.12F };
    float derivative_cutoff_hz{ 1.0F };
};

class OneEuroFilter2D {
public:
    explicit OneEuroFilter2D(OneEuroConfig config = {});

    Vec2 update(Vec2 value, std::int64_t timestamp_ms);
    void reset();

private:
    OneEuroConfig config_;
    std::optional<Vec2> previous_raw_;
    std::optional<Vec2> previous_filtered_;
    Vec2 filtered_derivative_{};
    std::optional<std::int64_t> previous_timestamp_ms_;
};

struct HandSignalConfig {
    OneEuroConfig filter;
    std::int64_t display_hold_ms{ 100 };
};

struct StabilizedFingertip {
    Vec2 raw{};
    Vec2 filtered{};
    bool confirmed_this_frame{ false };
};

class HandSignalStabilizer {
public:
    explicit HandSignalStabilizer(HandSignalConfig config = {});

    std::optional<StabilizedFingertip> update(std::optional<Vec2> observation,
                                               std::int64_t timestamp_ms);
    void reset();

private:
    HandSignalConfig config_;
    OneEuroFilter2D filter_;
    std::optional<StabilizedFingertip> last_value_;
    std::optional<std::int64_t> last_observation_timestamp_ms_;
};

struct DepthSignalConfig {
    std::size_t median_window_size{ 5U };
    float max_jump_mm{ 80.0F };
    std::size_t invalid_reset_frames{ 3U };
};

class DepthSignalStabilizer {
public:
    explicit DepthSignalStabilizer(DepthSignalConfig config = {});

    std::optional<float> update(std::optional<float> depth_mm);
    void reset();

private:
    DepthSignalConfig config_;
    std::deque<float> history_;
    std::size_t invalid_frames_{};
    std::size_t rejected_jump_frames_{};
};

class DepthFreshnessGate {
public:
    explicit DepthFreshnessGate(std::size_t invalid_reset_frames);

    bool update(std::optional<float> raw_depth_mm);
    void reset();

private:
    std::size_t invalid_reset_frames_{};
    std::size_t consecutive_invalid_frames_{};
    bool stale_{ false };
};

}  // namespace aerial_touch
