#pragma once

#include "aerial_touch/types.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace aerial_touch {

struct CalibrationSamplingConfig {
    std::size_t required_samples{ 18U };
    float mad_multiplier{ 3.5F };
    float minimum_outlier_threshold_mm{ 2.0F };
};

struct CalibrationSampleResult {
    Vec3 point{};
    Vec3 spread{};
    std::size_t accepted_samples{};
    std::size_t total_samples{};
};

class CalibrationSampleCollector {
public:
    explicit CalibrationSampleCollector(CalibrationSamplingConfig config = {});

    bool add(Vec3 sample);
    std::optional<CalibrationSampleResult> result() const;
    std::size_t sample_count() const;
    void clear();

private:
    CalibrationSamplingConfig config_;
    std::vector<Vec3> samples_;
};

}  // namespace aerial_touch
