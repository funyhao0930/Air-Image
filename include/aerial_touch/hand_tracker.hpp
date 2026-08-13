#pragma once

#include "aerial_touch/types.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace aerial_touch {

struct HandObservation {
    bool detected{ false };
    int landmark_count{ 0 };
    std::array<Vec3, 21> landmarks{};
};

class HandTracker {
public:
    HandTracker(const std::filesystem::path& dll_path, const std::filesystem::path& model_path);
    ~HandTracker();

    HandTracker(const HandTracker&) = delete;
    HandTracker& operator=(const HandTracker&) = delete;
    HandTracker(HandTracker&&) noexcept;
    HandTracker& operator=(HandTracker&&) noexcept;

    bool available() const;
    const std::string& error() const;
    HandObservation detect_rgb(const unsigned char* rgb,
                               int width,
                               int height,
                               int stride_bytes,
                               std::int64_t timestamp_ms);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace aerial_touch
