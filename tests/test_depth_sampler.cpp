#include "aerial_touch/depth_sampler.hpp"

#include <cassert>
#include <cstdint>
#include <vector>

bool run_interaction_core_tests();
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

    return run_interaction_core_tests() && run_signal_stability_tests() ? 0 : 1;
}
