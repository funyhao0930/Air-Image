#include "aerial_touch/hand_tracker.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if(argc != 4) {
        std::cerr << "Usage: aerial_touch_bridge_smoke <bridge.dll> <hand_landmarker.task> <hand.jpg>\n";
        return 2;
    }

    aerial_touch::HandTracker tracker(argv[1], argv[2]);
    if(!tracker.available()) {
        std::cerr << tracker.error() << '\n';
        return 1;
    }

    constexpr int width = 64;
    constexpr int height = 64;
    std::vector<std::uint8_t> black_rgb(static_cast<std::size_t>(width * height * 3), 0U);
    const auto observation = tracker.detect_rgb(black_rgb.data(), width, height, width * 3, 1);
    if(!tracker.error().empty()) {
        std::cerr << tracker.error() << '\n';
        return 1;
    }
    if(observation.detected || observation.landmark_count != 0) {
        std::cerr << "Blank RGB frame unexpectedly contained a detected hand\n";
        return 1;
    }

    const cv::Mat bgr = cv::imread(argv[3], cv::IMREAD_COLOR);
    if(bgr.empty()) {
        std::cerr << "Could not decode hand test image\n";
        return 1;
    }
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    const auto hand = tracker.detect_rgb(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), 2);
    if(!tracker.error().empty()) {
        std::cerr << tracker.error() << '\n';
        return 1;
    }
    if(!hand.detected || hand.landmark_count != 21) {
        std::cerr << "Official MediaPipe hand image did not return 21 landmarks\n";
        return 1;
    }
    std::cout << "MediaPipe bridge safely returned no hand for blank RGB and 21 landmarks for the official test image\n";
    return 0;
}
