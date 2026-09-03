#pragma once

#include "aerial_touch/surface_plane.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace aerial_touch {

enum class SurfaceScanState {
    Ready,
    Collecting,
    Complete,
    Failed,
};

struct SurfaceScanConfig {
    std::int64_t minimum_duration_ms{ 1500 };
    std::int64_t timeout_ms{ 8000 };
    std::int64_t fit_interval_ms{ 250 };
    std::size_t maximum_samples{ 4000U };
    SurfacePlaneFitConfig plane_fit;
};

struct SurfaceScanProgress {
    SurfaceScanState state{ SurfaceScanState::Ready };
    std::size_t sample_count{};
    SurfacePlaneFitQuality quality{};
};

class SurfaceScanCollector {
public:
    explicit SurfaceScanCollector(SurfaceScanConfig config = {});

    void begin(std::int64_t timestamp_ms);
    SurfaceScanProgress add(const std::vector<Vec3>& samples, std::int64_t timestamp_ms);
    void clear();

    const SurfaceScanProgress& progress() const;
    const std::optional<SurfacePlane>& surface_plane() const;

private:
    SurfaceScanConfig config_;
    std::vector<Vec3> samples_;
    std::optional<std::int64_t> started_at_ms_;
    std::optional<std::int64_t> last_fit_at_ms_;
    std::optional<SurfacePlane> surface_plane_;
    SurfaceScanProgress progress_;
};

}  // namespace aerial_touch
