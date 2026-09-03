#include "aerial_touch/depth_sampler.hpp"

#include <cassert>
#include <cstdint>
#include <vector>

bool run_interaction_core_tests();
bool run_hud_text_tests();
bool run_signal_stability_tests();

int main() {
    const std::vector<std::uint16_t> depth{
        0, 1000, 1000, 1000, 0,
        1000, 1000, 65000, 1000, 1000,
        1000, 1000, 1000, 1000, 1000,
        1000, 1000, 1000, 1000, 1000,
        0, 1000, 1000, 1000, 0,
    };

    const auto sample = aerial_touch::sample_depth_median_mm(depth, 5, 5, 2, 2, 2, 1.0F);

    assert(sample.has_value());
    assert(*sample == 1000.0F);

    const std::vector<std::uint16_t> invalid_depth(25, 0U);
    const auto invalid_sample = aerial_touch::sample_depth_median_mm(invalid_depth, 5, 5, 2, 2, 2, 1.0F);
    assert(!invalid_sample.has_value());

    std::vector<std::uint16_t> annulus_depth(81U, 1000U);
    annulus_depth[static_cast<std::size_t>(4 * 9 + 4)] = 850U;
    const auto annulus = aerial_touch::sample_depth_annulus_mm(annulus_depth, 9, 9, 4, 4, 1, 3, 1.0F);
    assert(!annulus.empty());
    for(const auto& sample : annulus) {
        assert(!(sample.x == 4 && sample.y == 4));
        assert(sample.depth_mm == 1000.0F);
    }

    return run_interaction_core_tests() && run_hud_text_tests() && run_signal_stability_tests() ? 0 : 1;
}
