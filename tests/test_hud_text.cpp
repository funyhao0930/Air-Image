#include "aerial_touch/utf8_text.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

namespace {

bool chinese_hud_glyph_has_readable_pixel_height() {
    cv::Mat image(96, 256, CV_8UC3, cv::Scalar{ 0, 0, 0 });
    {
        aerial_touch::Utf8TextCanvas canvas(image);
        canvas.draw(u8"觸控", { 12, 12 });
    }

    cv::Mat bright_pixels;
    cv::inRange(image, cv::Scalar{ 240, 240, 240 }, cv::Scalar{ 255, 255, 255 }, bright_pixels);
    std::vector<cv::Point> points;
    cv::findNonZero(bright_pixels, points);
    return !points.empty() && cv::boundingRect(points).height >= 24;
}

}  // namespace

bool run_hud_text_tests() {
    return chinese_hud_glyph_has_readable_pixel_height();
}
