#include "aerial_touch/utf8_text.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cstring>
#include <utility>

#if defined(_WIN32)
#  define WIN32_LEAN_AND_MEAN
#  define NOMINMAX
#  include <windows.h>
#endif

namespace aerial_touch {
namespace {

#if defined(_WIN32)
std::wstring utf8_to_wide(const std::string& text) {
    if(text.empty()) {
        return {};
    }
    const int required = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()),
                                              nullptr, 0);
    if(required <= 0) {
        return {};
    }
    std::wstring result(static_cast<std::size_t>(required), L'\0');
    MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()), result.data(), required);
    return result;
}

COLORREF color_ref(const cv::Scalar& color) {
    const auto channel = [](const double value) {
        return static_cast<BYTE>(std::clamp(static_cast<int>(value), 0, 255));
    };
    return RGB(channel(color[2]), channel(color[1]), channel(color[0]));
}
#endif

}  // namespace

struct Utf8TextCanvas::Impl {
    explicit Impl(cv::Mat& target) : image(target) {
#if defined(_WIN32)
        if(image.empty() || image.type() != CV_8UC3) {
            return;
        }

        screen_dc = GetDC(nullptr);
        memory_dc = screen_dc != nullptr ? CreateCompatibleDC(screen_dc) : nullptr;
        if(memory_dc == nullptr) {
            return;
        }

        BITMAPINFO bitmap_info{};
        bitmap_info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bitmap_info.bmiHeader.biWidth = image.cols;
        bitmap_info.bmiHeader.biHeight = -image.rows;
        bitmap_info.bmiHeader.biPlanes = 1;
        bitmap_info.bmiHeader.biBitCount = 32;
        bitmap_info.bmiHeader.biCompression = BI_RGB;
        bitmap = CreateDIBSection(memory_dc, &bitmap_info, DIB_RGB_COLORS, &pixels, nullptr, 0);
        if(bitmap == nullptr || pixels == nullptr) {
            return;
        }
        previous_bitmap = SelectObject(memory_dc, bitmap);

        font = CreateFontW(-18, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                           CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH | FF_DONTCARE,
                           L"Microsoft JhengHei UI");
        if(font == nullptr) {
            return;
        }
        previous_font = SelectObject(memory_dc, font);
        SetBkMode(memory_dc, TRANSPARENT);

        cv::Mat bgra;
        cv::cvtColor(image, bgra, cv::COLOR_BGR2BGRA);
        std::memcpy(pixels, bgra.data, bgra.total() * bgra.elemSize());
        ready = true;
#endif
    }

    ~Impl() {
#if defined(_WIN32)
        if(ready) {
            cv::Mat bgra(image.rows, image.cols, CV_8UC4, pixels);
            cv::cvtColor(bgra, image, cv::COLOR_BGRA2BGR);
        }
        if(memory_dc != nullptr && previous_font != nullptr) {
            SelectObject(memory_dc, previous_font);
        }
        if(font != nullptr) {
            DeleteObject(font);
        }
        if(memory_dc != nullptr && previous_bitmap != nullptr) {
            SelectObject(memory_dc, previous_bitmap);
        }
        if(bitmap != nullptr) {
            DeleteObject(bitmap);
        }
        if(memory_dc != nullptr) {
            DeleteDC(memory_dc);
        }
        if(screen_dc != nullptr) {
            ReleaseDC(nullptr, screen_dc);
        }
#endif
    }

    void draw(const std::string& text, const cv::Point& origin, const cv::Scalar& color) {
#if defined(_WIN32)
        if(ready) {
            const std::wstring wide_text = utf8_to_wide(text);
            if(wide_text.empty()) {
                return;
            }
            static const std::array<cv::Point, 8> outline_offsets{ {
                { -1, -1 }, { 0, -1 }, { 1, -1 }, { -1, 0 },
                { 1, 0 },   { -1, 1 }, { 0, 1 },  { 1, 1 },
            } };
            SetTextColor(memory_dc, RGB(0, 0, 0));
            for(const auto& offset : outline_offsets) {
                TextOutW(memory_dc, origin.x + offset.x, origin.y + offset.y, wide_text.c_str(),
                         static_cast<int>(wide_text.size()));
            }
            SetTextColor(memory_dc, color_ref(color));
            TextOutW(memory_dc, origin.x, origin.y, wide_text.c_str(), static_cast<int>(wide_text.size()));
            return;
        }
#endif
        cv::putText(image, text, { origin.x, origin.y + 18 }, cv::FONT_HERSHEY_SIMPLEX, 0.58, { 0, 0, 0 }, 3,
                    cv::LINE_AA);
        cv::putText(image, text, { origin.x, origin.y + 18 }, cv::FONT_HERSHEY_SIMPLEX, 0.58, color, 1,
                    cv::LINE_AA);
    }

    cv::Mat& image;
    bool ready{ false };
#if defined(_WIN32)
    HDC screen_dc{ nullptr };
    HDC memory_dc{ nullptr };
    HBITMAP bitmap{ nullptr };
    HGDIOBJ previous_bitmap{ nullptr };
    HFONT font{ nullptr };
    HGDIOBJ previous_font{ nullptr };
    void* pixels{ nullptr };
#endif
};

Utf8TextCanvas::Utf8TextCanvas(cv::Mat& image) : impl_(std::make_unique<Impl>(image)) {}
Utf8TextCanvas::~Utf8TextCanvas() = default;

void Utf8TextCanvas::draw(const std::string& text, const cv::Point& origin, const cv::Scalar& color) {
    impl_->draw(text, origin, color);
}

void enable_utf8_console() {
#if defined(_WIN32)
    SetConsoleOutputCP(CP_UTF8);
#endif
}

void set_utf8_window_title(const char* window_name, const std::string& title) {
#if defined(_WIN32)
    const std::wstring wide_title = utf8_to_wide(title);
    if(const HWND window = FindWindowA(nullptr, window_name); window != nullptr && !wide_title.empty()) {
        SetWindowTextW(window, wide_title.c_str());
    }
#else
    (void)window_name;
    (void)title;
#endif
}

}  // namespace aerial_touch
