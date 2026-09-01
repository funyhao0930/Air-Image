#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <memory>
#include <string>

namespace aerial_touch {

class Utf8TextCanvas {
public:
    explicit Utf8TextCanvas(cv::Mat& image);
    ~Utf8TextCanvas();

    Utf8TextCanvas(const Utf8TextCanvas&) = delete;
    Utf8TextCanvas& operator=(const Utf8TextCanvas&) = delete;

    void draw(const std::string& text, const cv::Point& origin, const cv::Scalar& color = { 255, 255, 255 });

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

void enable_utf8_console();
void set_utf8_window_title(const char* window_name, const std::string& title);

}  // namespace aerial_touch
