#pragma once

namespace aerial_touch {

enum class AlignmentMode {
    Hardware,
    Software,
};

constexpr AlignmentMode choose_alignment_mode(const bool hardware_d2c_available) {
    return hardware_d2c_available ? AlignmentMode::Hardware : AlignmentMode::Software;
}

}  // namespace aerial_touch
