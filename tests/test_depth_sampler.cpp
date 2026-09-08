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

bool depth_annulus_excludes_the_centre_disc() {
    std::vector<std::uint16_t> annulus_depth(81U, 1000U);
    annulus_depth[static_cast<std::size_t>(4 * 9 + 4)] = 850U;
    const auto annulus = aerial_touch::sample_depth_annulus_mm(annulus_depth, 9, 9, 4, 4, 1, 3, 1.0F);
    if(!check(!annulus.empty(), "annulus returns samples")) {
        return false;
    }
    for(const auto& sample : annulus) {
        if(!check(!(sample.x == 4 && sample.y == 4), "annulus skips the centre pixel")
           || !check(sample.depth_mm == 1000.0F, "annulus reports the ring depth")) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main() {
    bool passed = depth_sampler_takes_the_median_of_valid_pixels();
    passed = depth_annulus_excludes_the_centre_disc() && passed;
    passed = run_interaction_core_tests() && passed;
    passed = run_hud_text_tests() && passed;
    passed = run_signal_stability_tests() && passed;
    return passed ? 0 : 1;
}
