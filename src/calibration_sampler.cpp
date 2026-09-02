#include "aerial_touch/calibration_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aerial_touch {
namespace {

float median(std::vector<float> values) {
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
    std::nth_element(values.begin(), middle, values.end());
    if(values.size() % 2U == 1U) {
        return *middle;
    }
    const float upper = *middle;
    return (*std::max_element(values.begin(), middle) + upper) / 2.0F;
}

std::vector<float> component_values(const std::vector<Vec3>& samples, const int component) {
    std::vector<float> values;
    values.reserve(samples.size());
    for(const auto& sample : samples) {
        values.push_back(component == 0 ? sample.x : component == 1 ? sample.y : sample.z);
    }
    return values;
}

float median_absolute_deviation(const std::vector<float>& values, const float center) {
    std::vector<float> deviations;
    deviations.reserve(values.size());
    for(const float value : values) {
        deviations.push_back(std::fabs(value - center));
    }
    return median(std::move(deviations));
}

}  // namespace

CalibrationSampleCollector::CalibrationSampleCollector(const CalibrationSamplingConfig config) : config_(config) {
    if(config_.required_samples < 15U || config_.required_samples > 20U
       || !std::isfinite(config_.mad_multiplier) || config_.mad_multiplier <= 0.0F
       || !std::isfinite(config_.minimum_outlier_threshold_mm)
       || config_.minimum_outlier_threshold_mm <= 0.0F) {
        throw std::invalid_argument(u8"校正取樣設定無效");
    }
}

bool CalibrationSampleCollector::add(const Vec3 sample) {
    if(!std::isfinite(sample.x) || !std::isfinite(sample.y) || !std::isfinite(sample.z) || sample.z <= 0.0F) {
        return false;
    }
    if(samples_.size() >= 20U) {
        return false;
    }
    samples_.push_back(sample);
    return true;
}

std::optional<CalibrationSampleResult> CalibrationSampleCollector::result() const {
    if(samples_.size() < config_.required_samples) {
        return std::nullopt;
    }

    const auto x_values = component_values(samples_, 0);
    const auto y_values = component_values(samples_, 1);
    const auto z_values = component_values(samples_, 2);
    const Vec3 center{ median(x_values), median(y_values), median(z_values) };
    const Vec3 limits{
        std::max(config_.minimum_outlier_threshold_mm,
                 config_.mad_multiplier * median_absolute_deviation(x_values, center.x)),
        std::max(config_.minimum_outlier_threshold_mm,
                 config_.mad_multiplier * median_absolute_deviation(y_values, center.y)),
        std::max(config_.minimum_outlier_threshold_mm,
                 config_.mad_multiplier * median_absolute_deviation(z_values, center.z)),
    };

    std::vector<Vec3> accepted;
    accepted.reserve(samples_.size());
    for(const auto& sample : samples_) {
        if(std::fabs(sample.x - center.x) <= limits.x && std::fabs(sample.y - center.y) <= limits.y
           && std::fabs(sample.z - center.z) <= limits.z) {
            accepted.push_back(sample);
        }
    }
    if(accepted.size() < config_.required_samples) {
        return std::nullopt;
    }

    const auto accepted_x = component_values(accepted, 0);
    const auto accepted_y = component_values(accepted, 1);
    const auto accepted_z = component_values(accepted, 2);
    const auto spread = [](const std::vector<float>& values) {
        const auto extrema = std::minmax_element(values.begin(), values.end());
        return *extrema.second - *extrema.first;
    };
    return CalibrationSampleResult{
        { median(accepted_x), median(accepted_y), median(accepted_z) },
        { spread(accepted_x), spread(accepted_y), spread(accepted_z) },
        accepted.size(),
        samples_.size(),
    };
}

std::size_t CalibrationSampleCollector::sample_count() const {
    return samples_.size();
}

void CalibrationSampleCollector::clear() {
    samples_.clear();
}

}  // namespace aerial_touch
