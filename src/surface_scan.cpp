#include "aerial_touch/surface_scan.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aerial_touch {
namespace {

bool finite(const Vec3 sample) {
    return std::isfinite(sample.x) && std::isfinite(sample.y) && std::isfinite(sample.z);
}

bool valid_config(const SurfaceScanConfig& config) {
    return config.minimum_duration_ms >= 0 && config.timeout_ms >= config.minimum_duration_ms
           && config.fit_interval_ms >= 0
           && config.maximum_samples >= config.plane_fit.minimum_samples;
}

}  // namespace

SurfaceScanCollector::SurfaceScanCollector(const SurfaceScanConfig config) : config_(config) {
    if(!valid_config(config_)) {
        throw std::invalid_argument(u8"表面掃描設定無效");
    }
}

std::size_t SurfaceScanCollector::next_reservoir_slot() {
    // Deterministic LCG (Numerical Recipes constants): the scan must replay identically for a given
    // input sequence so the tests and any recorded session stay reproducible.
    random_state_ = random_state_ * 1664525U + 1013904223U;
    return static_cast<std::size_t>((random_state_ >> 16U) % seen_samples_);
}

void SurfaceScanCollector::begin(const std::int64_t timestamp_ms) {
    samples_.clear();
    seen_samples_ = 0U;
    random_state_ = 0x9E3779B97F4A7C15ULL;
    surface_plane_.reset();
    started_at_ms_ = timestamp_ms;
    last_fit_at_ms_.reset();
    progress_ = { SurfaceScanState::Collecting, 0U, {} };
}

SurfaceScanProgress SurfaceScanCollector::add(const std::vector<Vec3>& samples, const std::int64_t timestamp_ms) {
    if(progress_.state != SurfaceScanState::Collecting || !started_at_ms_.has_value()) {
        return progress_;
    }

    for(const Vec3 sample : samples) {
        if(!finite(sample)) {
            continue;
        }
        ++seen_samples_;
        if(samples_.size() < config_.maximum_samples) {
            samples_.push_back(sample);
            continue;
        }
        // Reservoir replacement rather than dropping everything past the cap. The cap fills within
        // about a second, so the old behaviour fitted the plane to the first moment of the sweep and
        // silently discarded the rest of the area the operator was asked to cover.
        const std::size_t slot = next_reservoir_slot();
        if(slot < samples_.size()) {
            samples_[slot] = sample;
        }
    }
    progress_.sample_count = static_cast<std::size_t>(seen_samples_);
    progress_.quality.total_samples = samples_.size();
    const std::int64_t elapsed_ms = std::max<std::int64_t>(0, timestamp_ms - *started_at_ms_);
    const bool timestamp_moved_backward = last_fit_at_ms_.has_value() && timestamp_ms < *last_fit_at_ms_;
    const bool can_fit = elapsed_ms >= config_.minimum_duration_ms
                         && (!last_fit_at_ms_.has_value() || timestamp_moved_backward
                             || timestamp_ms - *last_fit_at_ms_ >= config_.fit_interval_ms);
    if(can_fit) {
        last_fit_at_ms_ = timestamp_ms;
        SurfacePlaneFitQuality quality;
        const auto plane = SurfacePlane::fit(samples_, config_.plane_fit, &quality);
        // Report the attempt's quality either way. A sweep that is failing only on extent looks
        // identical to one failing on noise unless the operator can watch the spread grow.
        progress_.quality = quality;
        if(plane.has_value()) {
            surface_plane_ = *plane;
            progress_ = { SurfaceScanState::Complete, static_cast<std::size_t>(seen_samples_), quality };
            return progress_;
        }
    }
    if(elapsed_ms > config_.timeout_ms) {
        progress_.state = SurfaceScanState::Failed;
    }
    return progress_;
}

void SurfaceScanCollector::clear() {
    samples_.clear();
    seen_samples_ = 0U;
    random_state_ = 0x9E3779B97F4A7C15ULL;
    started_at_ms_.reset();
    last_fit_at_ms_.reset();
    surface_plane_.reset();
    progress_ = {};
}

const SurfaceScanProgress& SurfaceScanCollector::progress() const {
    return progress_;
}

const std::optional<SurfacePlane>& SurfaceScanCollector::surface_plane() const {
    return surface_plane_;
}

}  // namespace aerial_touch
