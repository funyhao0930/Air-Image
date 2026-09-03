#pragma once

#include "aerial_touch/types.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace aerial_touch {

struct SurfacePlaneFitConfig {
    std::size_t minimum_samples{ 60U };
    float minimum_inlier_ratio{ 0.70F };
    float inlier_threshold_mm{ 6.0F };
    float maximum_rms_residual_mm{ 4.0F };
};

struct SurfacePlaneFitQuality {
    std::size_t total_samples{};
    std::size_t inlier_samples{};
    float rms_residual_mm{};
};

class SurfacePlane {
public:
    static std::optional<SurfacePlane> fit(const std::vector<Vec3>& samples,
                                           SurfacePlaneFitConfig config = {},
                                           SurfacePlaneFitQuality* quality = nullptr);

    std::optional<Vec3> project_to_surface(Vec3 point) const;
    std::optional<Vec3> intersect_ray(Vec3 origin, Vec3 direction) const;
    float signed_distance(Vec3 point) const;

private:
    SurfacePlane(Vec3 origin, Vec3 normal);

    Vec3 origin_{};
    Vec3 normal_{};
};

}  // namespace aerial_touch
