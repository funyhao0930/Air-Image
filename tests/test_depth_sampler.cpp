#include "aerial_touch/depth_sampler.hpp"

#include <cstdint>
#include <iostream>
#include <vector>

bool run_interaction_core_tests();
bool run_hud_text_tests();
bool run_signal_stability_tests();

namespace {

// NOT assert(): the documented build is the Release preset, which defines NDEBUG and would compile
// every assertion in this file away, leaving the depth-sampler cases silently untested.
bool check(const bool condition, const char* what) {
    if(!condition) {
        std::cerr << "FAILED: " << what << "\n";
    }
    return condition;
}

bool depth_sampler_takes_the_median_of_valid_pixels() {
    const std::vector<std::uint16_t> depth{
        0, 1000, 1000, 1000, 0,
        1000, 1000, 65000, 1000, 1000,
        1000, 1000, 1000, 1000, 1000,
        1000, 1000, 1000, 1000, 1000,
        0, 1000, 1000, 1000, 0,
    };

    const auto sample = aerial_touch::sample_depth_median_mm(depth, 5, 5, 2, 2, 2, 1.0F);
    bool passed = check(sample.has_value(), "depth median returns a value");
    passed = check(passed && *sample == 1000.0F, "depth median rejects the single outlier") && passed;

    const std::vector<std::uint16_t> invalid_depth(25, 0U);
    const auto invalid_sample = aerial_touch::sample_depth_median_mm(invalid_depth, 5, 5, 2, 2, 2, 1.0F);
    return check(!invalid_sample.has_value(), "all-zero depth yields no sample") && passed;
}

bool surface_grid_stays_inside_its_disc_and_skips_excluded_points() {
    std::vector<std::uint16_t> depth(81U, 1000U);
    depth[static_cast<std::size_t>(4 * 9 + 4)] = 850U;  // the pointing fingertip itself

    aerial_touch::SurfaceSampleRegion region;
    region.center_x = 4;
    region.center_y = 4;
    region.radius_px = 4;
    region.stride_px = 1;

    const std::vector<aerial_touch::Vec2> excluded{ { 4.0F, 4.0F } };
    const auto samples =
        aerial_touch::sample_depth_surface_grid_mm(depth, 9, 9, region, 1.0F, excluded, 1.5F);
    if(!check(!samples.empty(), "surface grid returns samples")) {
        return false;
    }
    for(const auto& sample : samples) {
        const int dx = sample.x - 4;
        const int dy = sample.y - 4;
        if(!check(dx * dx + dy * dy <= 16, "surface grid stays inside the disc")
           || !check(!(sample.x == 4 && sample.y == 4), "surface grid skips the excluded point")
           || !check(sample.depth_mm == 1000.0F, "surface grid reports the surface depth")) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main() {
    bool passed = depth_sampler_takes_the_median_of_valid_pixels();
    passed = surface_grid_stays_inside_its_disc_and_skips_excluded_points() && passed;
    passed = run_interaction_core_tests() && passed;
    passed = run_hud_text_tests() && passed;
    passed = run_signal_stability_tests() && passed;
    return passed ? 0 : 1;
}
