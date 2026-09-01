#include "aerial_touch/settings_window.hpp"

#ifdef _WIN32

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>
#include <commctrl.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cwchar>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace aerial_touch {

namespace {

constexpr int kFieldCount = 10;
constexpr int kFirstEditId = 1000;
constexpr int kFirstSliderId = 1100;
constexpr int kRestoreButtonId = 1201;
constexpr int kCancelButtonId = 1202;
constexpr int kApplyButtonId = 1203;
constexpr int kStatusId = 1300;
constexpr int kPathId = 1301;
constexpr int kFirstErrorId = 1400;

struct FieldSpec {
    const wchar_t* label;
    const wchar_t* unit;
    double slider_min;
    double slider_max;
    double slider_scale;
    bool integral;
};

const std::array<FieldSpec, kFieldCount>& field_specs() {
    static const std::array<FieldSpec, kFieldCount> specs{ {
        { L"觸控門檻\r\n接近至此距離觸發", L"mm", 0.0, 30.0, 10.0, false },
        { L"離開門檻\r\n超過此距離重新啟用", L"mm", 5.0, 60.0, 10.0, false },
        { L"最低接近速度\r\n防止慢速誤觸", L"mm/s", 0.0, 500.0, 10.0, false },
        { L"追蹤逾時\r\n遺失手部後鎖定時間", L"ms", 50.0, 1000.0, 1.0, true },
        { L"深度取樣半徑\r\nROI 中位數範圍", L"px", 0.0, 8.0, 1.0, true },
        { L"校正點最小距離\r\n避免三點太接近", L"mm", 20.0, 300.0, 10.0, false },
        { L"按鍵寬度", L"mm", 10.0, 80.0, 10.0, false },
        { L"按鍵高度", L"mm", 10.0, 80.0, 10.0, false },
        { L"水平間距", L"mm", 0.0, 30.0, 10.0, false },
        { L"垂直間距", L"mm", 0.0, 30.0, 10.0, false },
    } };
    return specs;
}

double field_value(const AppConfig& config, const int index) {
    switch(index) {
    case 0:
        return config.touch.touch_threshold_mm;
    case 1:
        return config.touch.release_threshold_mm;
    case 2:
        return config.touch.min_approach_velocity_mm_s;
    case 3:
        return static_cast<double>(config.touch.tracking_timeout_ms);
    case 4:
        return static_cast<double>(config.depth.sample_radius);
    case 5:
        return config.calibration.minimum_point_distance_mm;
    case 6:
        return config.keypad.key_width_mm;
    case 7:
        return config.keypad.key_height_mm;
    case 8:
        return config.keypad.horizontal_gap_mm;
    case 9:
        return config.keypad.vertical_gap_mm;
    default:
        return 0.0;
    }
}

void set_field_value(AppConfig& config, const int index, const double value) {
    switch(index) {
    case 0:
        config.touch.touch_threshold_mm = static_cast<float>(value);
        break;
    case 1:
        config.touch.release_threshold_mm = static_cast<float>(value);
        break;
    case 2:
        config.touch.min_approach_velocity_mm_s = static_cast<float>(value);
        break;
    case 3:
        config.touch.tracking_timeout_ms = static_cast<std::int64_t>(value);
        break;
    case 4:
        config.depth.sample_radius = static_cast<int>(value);
        break;
    case 5:
        config.calibration.minimum_point_distance_mm = static_cast<float>(value);
        break;
    case 6:
        config.keypad.key_width_mm = static_cast<float>(value);
        break;
    case 7:
        config.keypad.key_height_mm = static_cast<float>(value);
        break;
    case 8:
        config.keypad.horizontal_gap_mm = static_cast<float>(value);
        break;
    case 9:
        config.keypad.vertical_gap_mm = static_cast<float>(value);
        break;
    default:
        break;
    }
}

bool configs_equal(const AppConfig& left, const AppConfig& right) {
    return left.depth.sample_radius == right.depth.sample_radius
           && left.touch.touch_threshold_mm == right.touch.touch_threshold_mm
           && left.touch.release_threshold_mm == right.touch.release_threshold_mm
           && left.touch.min_approach_velocity_mm_s == right.touch.min_approach_velocity_mm_s
           && left.touch.tracking_timeout_ms == right.touch.tracking_timeout_ms
           && left.keypad.key_width_mm == right.keypad.key_width_mm
           && left.keypad.key_height_mm == right.keypad.key_height_mm
           && left.keypad.horizontal_gap_mm == right.keypad.horizontal_gap_mm
           && left.keypad.vertical_gap_mm == right.keypad.vertical_gap_mm
           && left.calibration.minimum_point_distance_mm == right.calibration.minimum_point_distance_mm;
}

std::wstring trim(std::wstring value) {
    const auto first = value.find_first_not_of(L" \t\r\n");
    if(first == std::wstring::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(L" \t\r\n");
    return value.substr(first, last - first + 1U);
}

std::optional<double> parse_number(const std::wstring& raw, const FieldSpec& spec, std::wstring& error) {
    const std::wstring value = trim(raw);
    if(value.empty()) {
        error = L"不可為空";
        return std::nullopt;
    }

    errno = 0;
    wchar_t* end = nullptr;
    const double parsed = std::wcstod(value.c_str(), &end);
    if(end == value.c_str() || *end != L'\0' || errno == ERANGE || !std::isfinite(parsed)) {
        error = L"請輸入有限數值";
        return std::nullopt;
    }
    if(!spec.integral && (parsed > static_cast<double>(std::numeric_limits<float>::max())
                          || parsed < -static_cast<double>(std::numeric_limits<float>::max()))) {
        error = L"數值超出支援範圍";
        return std::nullopt;
    }
    if(spec.integral && std::floor(parsed) != parsed) {
        error = L"請輸入整數";
        return std::nullopt;
    }
    if(spec.integral && (parsed > static_cast<double>(std::numeric_limits<std::int64_t>::max())
                         || parsed < static_cast<double>(std::numeric_limits<std::int64_t>::min()))) {
        error = L"整數超出支援範圍";
        return std::nullopt;
    }
    return parsed;
}

std::wstring format_value(const double value, const bool integral) {
    std::wostringstream output;
    if(integral) {
        output << static_cast<std::int64_t>(value);
    }
    else {
        output << std::fixed << std::setprecision(1) << value;
    }
    return output.str();
}

int slider_position(const double value, const FieldSpec& spec) {
    const double clamped = std::clamp(value, spec.slider_min, spec.slider_max);
    return static_cast<int>(std::lround(clamped * spec.slider_scale));
}

std::wstring utf8_to_wide(const std::string& text) {
    if(text.empty()) {
        return {};
    }
    const int size = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()),
                                         nullptr, 0);
    if(size <= 0) {
        return L"操作失敗";
    }
    std::wstring result(static_cast<std::size_t>(size), L'\0');
    MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()), result.data(), size);
    return result;
}

std::wstring window_text(const HWND window) {
    const int length = GetWindowTextLengthW(window);
    std::wstring result(static_cast<std::size_t>(std::max(0, length)) + 1U, L'\0');
    if(length > 0) {
        GetWindowTextW(window, result.data(), length + 1);
    }
    result.resize(static_cast<std::size_t>(std::max(0, length)));
    return result;
}

void set_window_text(const HWND window, const std::wstring& text) {
    SetWindowTextW(window, text.c_str());
}

}  // namespace

PreviewZone classify_preview_zone(const std::optional<float> distance_mm,
                                  const float touch_threshold_mm,
                                  const float release_threshold_mm) {
    if(!distance_mm.has_value() || !std::isfinite(*distance_mm)) {
        return PreviewZone::Unknown;
    }
    if(*distance_mm <= touch_threshold_mm) {
        return PreviewZone::Touch;
    }
    if(*distance_mm >= release_threshold_mm) {
        return PreviewZone::Release;
    }
    return PreviewZone::Hold;
}

struct SettingsWindow::Impl {
    HWND hwnd{};
    AppConfig applied_config{};
    std::filesystem::path config_path;
    ApplyCallback apply_callback;
    SettingsPreview preview{};
    std::array<HWND, kFieldCount> edits{};
    std::array<HWND, kFieldCount> sliders{};
    std::array<HWND, kFieldCount> errors{};
    HWND status{};
    HWND path_label{};
    HWND apply_button{};
    HFONT font{};
    HFONT small_font{};
    HFONT heading_font{};
    HBRUSH background_brush{};
    HBRUSH edit_brush{};
    bool syncing{ false };

    static constexpr COLORREF kBackground = RGB(7, 18, 21);
    static constexpr COLORREF kPanel = RGB(13, 29, 32);
    static constexpr COLORREF kEdit = RGB(8, 21, 24);
    static constexpr COLORREF kText = RGB(233, 242, 241);
    static constexpr COLORREF kMuted = RGB(139, 165, 163);
    static constexpr COLORREF kCyan = RGB(97, 228, 220);
    static constexpr COLORREF kAmber = RGB(255, 202, 120);
    static constexpr COLORREF kRed = RGB(255, 130, 123);

    static ATOM register_class(const HINSTANCE instance) {
        static ATOM atom = 0;
        if(atom != 0) {
            return atom;
        }
        WNDCLASSEXW window_class{};
        window_class.cbSize = sizeof(window_class);
        window_class.hInstance = instance;
        window_class.lpfnWndProc = &Impl::window_proc;
        window_class.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512));
        window_class.lpszClassName = L"AerialTouchSettingsWindow";
        atom = RegisterClassExW(&window_class);
        return atom;
    }

    static void set_font(const HWND control, const HFONT font) {
        SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
    }

    HWND create_control(const wchar_t* class_name, const wchar_t* text, const DWORD style, const int x, const int y,
                        const int width, const int height, const int id) {
        HWND control = CreateWindowExW(0, class_name, text, style, x, y, width, height, hwnd,
                                       reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)), GetModuleHandleW(nullptr), nullptr);
        if(control != nullptr) {
            set_font(control, font);
        }
        return control;
    }

    void create_field(const int index, const int x, const int y) {
        const auto& spec = field_specs()[static_cast<std::size_t>(index)];
        const DWORD label_style = WS_CHILD | WS_VISIBLE | SS_LEFT;
        create_control(L"STATIC", spec.label, label_style, x, y, 125, 38, 0);

        const int slider_id = kFirstSliderId + index;
        sliders[static_cast<std::size_t>(index)] = create_control(TRACKBAR_CLASSW, L"", WS_CHILD | WS_VISIBLE | WS_TABSTOP,
                                                                    x + 130, y + 3, 205, 30, slider_id);
        SendMessageW(sliders[static_cast<std::size_t>(index)], TBM_SETRANGE, TRUE,
                     MAKELONG(0, static_cast<short>(std::lround(spec.slider_max * spec.slider_scale))));
        SendMessageW(sliders[static_cast<std::size_t>(index)], TBM_SETPAGESIZE, 0, 1);

        edits[static_cast<std::size_t>(index)] = create_control(
            L"EDIT", L"", WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL | ES_RIGHT,
            x + 340, y + 2, 58, 25, kFirstEditId + index);
        SendMessageW(edits[static_cast<std::size_t>(index)], EM_SETLIMITTEXT, 32, 0);
        create_control(L"STATIC", spec.unit, label_style, x + 404, y + 7, 42, 20, 0);
        errors[static_cast<std::size_t>(index)] = create_control(L"STATIC", L"", label_style, x + 130, y + 36, 290, 18,
                                                                  kFirstErrorId + index);
        if(errors[static_cast<std::size_t>(index)] != nullptr) {
            set_font(errors[static_cast<std::size_t>(index)], small_font);
            ShowWindow(errors[static_cast<std::size_t>(index)], SW_HIDE);
        }
    }

    void create_controls() {
        font = CreateFontW(-16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                           CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
        small_font = CreateFontW(-12, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                                 CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
        heading_font = CreateFontW(-18, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                                   CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
        background_brush = CreateSolidBrush(kBackground);
        edit_brush = CreateSolidBrush(kEdit);

        path_label = create_control(L"STATIC", L"", WS_CHILD | WS_VISIBLE | SS_LEFT, 24, 694, 500, 22, kPathId);
        set_font(path_label, small_font);
        status = create_control(L"STATIC", L"", WS_CHILD | WS_VISIBLE | SS_LEFT, 24, 666, 530, 25, kStatusId);
        set_font(status, small_font);

        for(int index = 0; index < 4; ++index) {
            create_field(index, 35, 105 + index * 58);
        }
        create_field(4, 35, 475);
        create_field(5, 35, 533);
        for(int index = 6; index < kFieldCount; ++index) {
            create_field(index, 480, 475 + (index - 6) * 58);
        }

        HWND restore = create_control(L"BUTTON", L"恢復預設值", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
                                      560, 685, 105, 32, kRestoreButtonId);
        HWND cancel = create_control(L"BUTTON", L"取消", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
                                     675, 685, 75, 32, kCancelButtonId);
        apply_button = create_control(L"BUTTON", L"套用並儲存", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_DEFPUSHBUTTON,
                                      760, 685, 120, 32, kApplyButtonId);
        set_font(restore, font);
        set_font(cancel, font);
        set_font(apply_button, font);
        load_config_to_controls(applied_config);
        const std::wstring path_text = std::wstring(L"設定檔：") + config_path.wstring();
        set_window_text(path_label, path_text);
    }

    void set_field_error(const int index, const std::wstring& error) {
        HWND error_control = errors[static_cast<std::size_t>(index)];
        set_window_text(error_control, error);
        ShowWindow(error_control, error.empty() ? SW_HIDE : SW_SHOW);
    }

    void set_status(const std::wstring& message, const COLORREF color) {
        set_window_text(status, message);
        InvalidateRect(status, nullptr, TRUE);
        status_color = color;
    }

    COLORREF status_color{ kMuted };

    void load_config_to_controls(const AppConfig& config) {
        syncing = true;
        for(int index = 0; index < kFieldCount; ++index) {
            const auto& spec = field_specs()[static_cast<std::size_t>(index)];
            const double value = field_value(config, index);
            set_window_text(edits[static_cast<std::size_t>(index)], format_value(value, spec.integral));
            SendMessageW(sliders[static_cast<std::size_t>(index)], TBM_SETPOS, TRUE, slider_position(value, spec));
            set_field_error(index, {});
        }
        syncing = false;
        refresh_validation();
    }

    bool read_candidate(AppConfig& candidate) {
        candidate = applied_config;
        bool valid = true;
        for(int index = 0; index < kFieldCount; ++index) {
            std::wstring error;
            const auto value = parse_number(window_text(edits[static_cast<std::size_t>(index)]),
                                            field_specs()[static_cast<std::size_t>(index)], error);
            set_field_error(index, error);
            if(!value.has_value()) {
                valid = false;
                continue;
            }
            if(index == 4 && (*value > static_cast<double>(std::numeric_limits<int>::max())
                              || *value < static_cast<double>(std::numeric_limits<int>::min()))) {
                set_field_error(index, L"整數超出支援範圍");
                valid = false;
                continue;
            }
            set_field_value(candidate, index, *value);
        }
        if(!valid) {
            return false;
        }

        try {
            validate_app_config(candidate);
        }
        catch(const std::exception& error) {
            if(candidate.touch.touch_threshold_mm < 0.0F) {
                set_field_error(0, L"不可小於 0");
            }
            else if(candidate.touch.release_threshold_mm <= candidate.touch.touch_threshold_mm) {
                set_field_error(1, L"必須大於觸控門檻");
            }
            else if(candidate.touch.min_approach_velocity_mm_s < 0.0F) {
                set_field_error(2, L"不可小於 0");
            }
            else if(candidate.touch.tracking_timeout_ms <= 0) {
                set_field_error(3, L"必須大於 0");
            }
            else if(candidate.depth.sample_radius < 0) {
                set_field_error(4, L"不可小於 0");
            }
            else if(candidate.calibration.minimum_point_distance_mm <= 0.0F) {
                set_field_error(5, L"必須大於 0");
            }
            else if(candidate.keypad.key_width_mm <= 0.0F) {
                set_field_error(6, L"必須大於 0");
            }
            else if(candidate.keypad.key_height_mm <= 0.0F) {
                set_field_error(7, L"必須大於 0");
            }
            else if(candidate.keypad.horizontal_gap_mm < 0.0F) {
                set_field_error(8, L"不可小於 0");
            }
            else if(candidate.keypad.vertical_gap_mm < 0.0F) {
                set_field_error(9, L"不可小於 0");
            }
            else {
                set_status(utf8_to_wide(error.what()), kRed);
            }
            return false;
        }
        return true;
    }

    void refresh_validation() {
        AppConfig candidate;
        const bool valid = read_candidate(candidate);
        const bool dirty = valid && !configs_equal(candidate, applied_config);
        EnableWindow(apply_button, valid && dirty);
        if(!valid) {
            set_status(L"請修正標示欄位後再套用", kRed);
        }
        else if(dirty) {
            set_status(L"設定有效 · 可套用", kCyan);
        }
        else {
            set_status(L"尚未修改", kMuted);
        }
        InvalidateRect(hwnd, nullptr, FALSE);
    }

    void sync_slider_from_edit(const int index) {
        std::wstring error;
        const auto value = parse_number(window_text(edits[static_cast<std::size_t>(index)]),
                                        field_specs()[static_cast<std::size_t>(index)], error);
        if(value.has_value()) {
            SendMessageW(sliders[static_cast<std::size_t>(index)], TBM_SETPOS, TRUE,
                         slider_position(*value, field_specs()[static_cast<std::size_t>(index)]));
        }
    }

    void sync_edit_from_slider(const int index) {
        const auto& spec = field_specs()[static_cast<std::size_t>(index)];
        const int position = static_cast<int>(SendMessageW(sliders[static_cast<std::size_t>(index)], TBM_GETPOS, 0, 0));
        const double value = static_cast<double>(position) / spec.slider_scale;
        syncing = true;
        set_window_text(edits[static_cast<std::size_t>(index)], format_value(value, spec.integral));
        syncing = false;
        refresh_validation();
    }

    void handle_apply() {
        AppConfig candidate;
        if(!read_candidate(candidate)) {
            refresh_validation();
            return;
        }
        std::string error;
        if(!apply_callback || !apply_callback(candidate, error)) {
            set_status(error.empty() ? L"套用設定失敗" : utf8_to_wide(error), kRed);
            return;
        }
        applied_config = candidate;
        set_status(L"已套用並儲存", kCyan);
        refresh_validation();
    }

    void handle_cancel() {
        load_config_to_controls(applied_config);
        ShowWindow(hwnd, SW_HIDE);
    }

    void handle_restore_defaults() {
        load_config_to_controls(AppConfig{});
        set_status(L"已載入預設值，按套用並儲存才會生效", kAmber);
    }

    void draw_group(HDC dc, const RECT rect, const wchar_t* title, const wchar_t* subtitle) {
        HBRUSH panel_brush = CreateSolidBrush(kPanel);
        HPEN panel_pen = CreatePen(PS_SOLID, 1, RGB(44, 70, 72));
        HGDIOBJ old_brush = SelectObject(dc, panel_brush);
        HGDIOBJ old_pen = SelectObject(dc, panel_pen);
        RoundRect(dc, rect.left, rect.top, rect.right, rect.bottom, 12, 12);
        SelectObject(dc, old_pen);
        SelectObject(dc, old_brush);
        DeleteObject(panel_pen);
        DeleteObject(panel_brush);

        SetBkMode(dc, TRANSPARENT);
        SetTextColor(dc, kText);
        SelectObject(dc, heading_font);
        TextOutW(dc, rect.left + 15, rect.top + 12, title, static_cast<int>(wcslen(title)));
        SetTextColor(dc, kCyan);
        SelectObject(dc, small_font);
        TextOutW(dc, rect.left + 133, rect.top + 16, subtitle, static_cast<int>(wcslen(subtitle)));
    }

    void draw_preview(HDC dc) {
        const RECT preview_rect{ 480, 105, 865, 405 };
        HBRUSH brush = CreateSolidBrush(RGB(8, 21, 24));
        FillRect(dc, &preview_rect, brush);
        DeleteObject(brush);

        HPEN plane_pen = CreatePen(PS_SOLID, 1, RGB(75, 143, 139));
        HGDIOBJ old_pen = SelectObject(dc, plane_pen);
        POINT plane[4]{ { 525, 205 }, { 820, 170 }, { 820, 290 }, { 525, 325 } };
        Polygon(dc, plane, 4);
        SelectObject(dc, old_pen);
        DeleteObject(plane_pen);

        const PreviewZone zone = classify_preview_zone(preview.distance_mm, applied_config.touch.touch_threshold_mm,
                                                        applied_config.touch.release_threshold_mm);
        const COLORREF zone_color = zone == PreviewZone::Touch ? kRed
                                     : zone == PreviewZone::Release ? kCyan
                                     : zone == PreviewZone::Hold ? kAmber
                                                                  : kMuted;
        int finger_y = 150;
        if(preview.distance_mm.has_value()) {
            const double normalized = std::clamp(static_cast<double>(*preview.distance_mm) / 80.0, 0.0, 1.0);
            finger_y = 160 + static_cast<int>(normalized * 105.0);
        }
        HBRUSH finger_brush = CreateSolidBrush(zone_color);
        HGDIOBJ old_brush = SelectObject(dc, finger_brush);
        Ellipse(dc, 665, finger_y, 681, finger_y + 16);
        SelectObject(dc, old_brush);
        DeleteObject(finger_brush);

        HPEN distance_pen = CreatePen(PS_DASH, 1, kAmber);
        old_pen = SelectObject(dc, distance_pen);
        MoveToEx(dc, 673, finger_y + 16, nullptr);
        LineTo(dc, 673, 255);
        SelectObject(dc, old_pen);
        DeleteObject(distance_pen);

        SetBkMode(dc, TRANSPARENT);
        SetTextColor(dc, kMuted);
        SelectObject(dc, small_font);
        const wchar_t* plane_label = L"虛擬鍵盤平面";
        TextOutW(dc, 540, 300, plane_label, static_cast<int>(wcslen(plane_label)));

        std::wostringstream distance_text;
        if(preview.distance_mm.has_value()) {
            distance_text << L"目前 " << std::fixed << std::setprecision(1) << *preview.distance_mm << L" mm";
        }
        else {
            distance_text << L"目前無有效距離";
        }
        SetTextColor(dc, zone_color);
        SelectObject(dc, font);
        const std::wstring distance = distance_text.str();
        TextOutW(dc, 690, 212, distance.c_str(), static_cast<int>(distance.size()));

        SetTextColor(dc, kText);
        SelectObject(dc, small_font);
        const std::wstring tracking = preview.tracking_detected ? L"追蹤：已偵測" : L"追蹤：未偵測";
        const std::wstring key = std::wstring(L"按鍵：")
                                 + (preview.key.has_value() ? utf8_to_wide(*preview.key) : L"-");
        TextOutW(dc, 500, 350, tracking.c_str(), static_cast<int>(tracking.size()));
        TextOutW(dc, 620, 350, key.c_str(), static_cast<int>(key.size()));

        const std::wstring touch = preview.armed ? L"觸控：可觸發" : L"觸控：等待手指離開";
        SetTextColor(dc, preview.armed ? kCyan : kAmber);
        TextOutW(dc, 500, 375, touch.c_str(), static_cast<int>(touch.size()));
        SetTextColor(dc, kMuted);
        const std::wstring hint = std::wstring(L"觸控 ")
                                  + format_value(applied_config.touch.touch_threshold_mm, false)
                                  + L" mm   /   離開 " + format_value(applied_config.touch.release_threshold_mm, false) + L" mm";
        TextOutW(dc, 650, 375, hint.c_str(), static_cast<int>(hint.size()));
    }

    void paint() {
        PAINTSTRUCT paint_struct{};
        const HDC dc = BeginPaint(hwnd, &paint_struct);
        FillRect(dc, &paint_struct.rcPaint, background_brush);
        SetBkMode(dc, TRANSPARENT);
        SetTextColor(dc, kCyan);
        SelectObject(dc, small_font);
        const wchar_t* eyebrow = L"AERIAL TOUCH / TUNING CONSOLE";
        TextOutW(dc, 24, 18, eyebrow, static_cast<int>(wcslen(eyebrow)));
        SetTextColor(dc, kText);
        SelectObject(dc, heading_font);
        const wchar_t* title = L"參數設定";
        TextOutW(dc, 24, 38, title, static_cast<int>(wcslen(title)));
        SetTextColor(dc, kMuted);
        SelectObject(dc, small_font);
        const wchar_t* description = L"調整感測、觸控與鍵盤幾何；按下套用後於目前執行中生效。";
        TextOutW(dc, 155, 44, description, static_cast<int>(wcslen(description)));

        draw_group(dc, { 20, 75, 450, 440 }, L"觸控判定", L"TOUCH LOGIC");
        draw_group(dc, { 20, 450, 450, 650 }, L"深度與校正", L"DEPTH / CALIBRATION");
        draw_group(dc, { 465, 75, 880, 430 }, L"觸控距離預覽", L"LIVE MODEL");
        draw_group(dc, { 465, 450, 880, 650 }, L"鍵盤幾何", L"KEYPAD GEOMETRY");
        draw_preview(dc);
        EndPaint(hwnd, &paint_struct);
    }

    static LRESULT CALLBACK window_proc(const HWND window, const UINT message, const WPARAM w_param, const LPARAM l_param) {
        Impl* self = reinterpret_cast<Impl*>(GetWindowLongPtrW(window, GWLP_USERDATA));
        if(message == WM_NCCREATE) {
            const auto* create = reinterpret_cast<const CREATESTRUCTW*>(l_param);
            self = static_cast<Impl*>(create->lpCreateParams);
            self->hwnd = window;
            SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
        }
        if(self == nullptr) {
            return DefWindowProcW(window, message, w_param, l_param);
        }

        switch(message) {
        case WM_CREATE:
            self->create_controls();
            return 0;
        case WM_CLOSE:
            self->handle_cancel();
            return 0;
        case WM_PAINT:
            self->paint();
            return 0;
        case WM_ERASEBKGND:
            return 1;
        case WM_HSCROLL:
            for(int index = 0; index < kFieldCount; ++index) {
                if(reinterpret_cast<HWND>(l_param) == self->sliders[static_cast<std::size_t>(index)]) {
                    self->sync_edit_from_slider(index);
                    return 0;
                }
            }
            break;
        case WM_COMMAND: {
            const int id = LOWORD(w_param);
            const int notification = HIWORD(w_param);
            if(id >= kFirstEditId && id < kFirstEditId + kFieldCount && notification == EN_CHANGE
               && !self->syncing) {
                self->sync_slider_from_edit(id - kFirstEditId);
                self->refresh_validation();
                return 0;
            }
            if(id == kRestoreButtonId && notification == BN_CLICKED) {
                self->handle_restore_defaults();
                return 0;
            }
            if(id == kCancelButtonId && notification == BN_CLICKED) {
                self->handle_cancel();
                return 0;
            }
            if(id == kApplyButtonId && notification == BN_CLICKED) {
                self->handle_apply();
                return 0;
            }
            break;
        }
        case WM_CTLCOLORSTATIC: {
            const HDC dc = reinterpret_cast<HDC>(w_param);
            const HWND control = reinterpret_cast<HWND>(l_param);
            SetBkMode(dc, TRANSPARENT);
            SetBkColor(dc, kBackground);
            COLORREF text_color = control == self->status ? self->status_color : kText;
            for(const HWND error_control : self->errors) {
                if(control == error_control) {
                    text_color = kRed;
                    break;
                }
            }
            SetTextColor(dc, text_color);
            return reinterpret_cast<LRESULT>(self->background_brush);
        }
        case WM_CTLCOLOREDIT: {
            const HDC dc = reinterpret_cast<HDC>(w_param);
            SetBkColor(dc, kEdit);
            SetTextColor(dc, kText);
            return reinterpret_cast<LRESULT>(self->edit_brush);
        }
        case WM_DESTROY:
            self->hwnd = nullptr;
            return 0;
        default:
            break;
        }
        return DefWindowProcW(window, message, w_param, l_param);
    }

    bool create_window() {
        INITCOMMONCONTROLSEX common_controls{};
        common_controls.dwSize = sizeof(common_controls);
        common_controls.dwICC = ICC_BAR_CLASSES;
        InitCommonControlsEx(&common_controls);
        const HINSTANCE instance = GetModuleHandleW(nullptr);
        if(register_class(instance) == 0) {
            return false;
        }
        hwnd = CreateWindowExW(WS_EX_TOOLWINDOW, L"AerialTouchSettingsWindow", L"Aerial Touch · 參數設定",
                              WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_THICKFRAME,
                              CW_USEDEFAULT, CW_USEDEFAULT, 920, 780, nullptr, nullptr, instance, this);
        if(hwnd == nullptr) {
            return false;
        }
        RECT work_area{};
        SystemParametersInfoW(SPI_GETWORKAREA, 0, &work_area, 0);
        const int width = 920;
        const int height = 780;
        const int x = work_area.left + (work_area.right - work_area.left - width) / 2;
        const int y = work_area.top + (work_area.bottom - work_area.top - height) / 2;
        SetWindowPos(hwnd, HWND_TOP, x, y, width, height, SWP_NOACTIVATE);
        return true;
    }
};

SettingsWindow::SettingsWindow() = default;

SettingsWindow::~SettingsWindow() {
#ifdef _WIN32
    if(impl_ != nullptr) {
        if(impl_->hwnd != nullptr) {
            DestroyWindow(impl_->hwnd);
        }
        if(impl_->font != nullptr) {
            DeleteObject(impl_->font);
        }
        if(impl_->small_font != nullptr) {
            DeleteObject(impl_->small_font);
        }
        if(impl_->heading_font != nullptr) {
            DeleteObject(impl_->heading_font);
        }
        if(impl_->background_brush != nullptr) {
            DeleteObject(impl_->background_brush);
        }
        if(impl_->edit_brush != nullptr) {
            DeleteObject(impl_->edit_brush);
        }
        delete impl_;
    }
#endif
}

bool SettingsWindow::create(const AppConfig& config, std::filesystem::path config_path, ApplyCallback apply_callback) {
#ifdef _WIN32
    if(impl_ != nullptr) {
        return impl_->hwnd != nullptr;
    }
    impl_ = new Impl;
    impl_->applied_config = config;
    impl_->config_path = std::move(config_path);
    impl_->apply_callback = std::move(apply_callback);
    if(!impl_->create_window()) {
        delete impl_;
        impl_ = nullptr;
        return false;
    }
    return true;
#else
    static_cast<void>(config);
    static_cast<void>(config_path);
    static_cast<void>(apply_callback);
    return false;
#endif
}

void SettingsWindow::show() {
#ifdef _WIN32
    if(impl_ != nullptr && impl_->hwnd != nullptr) {
        impl_->load_config_to_controls(impl_->applied_config);
        ShowWindow(impl_->hwnd, SW_SHOWNORMAL);
        SetForegroundWindow(impl_->hwnd);
    }
#endif
}

void SettingsWindow::hide() {
#ifdef _WIN32
    if(impl_ != nullptr && impl_->hwnd != nullptr) {
        ShowWindow(impl_->hwnd, SW_HIDE);
    }
#endif
}

bool SettingsWindow::visible() const {
#ifdef _WIN32
    return impl_ != nullptr && impl_->hwnd != nullptr && IsWindowVisible(impl_->hwnd) != FALSE;
#else
    return false;
#endif
}

void SettingsWindow::process_messages() {
#ifdef _WIN32
    MSG message{};
    while(PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
        if(message.message == WM_QUIT) {
            PostQuitMessage(static_cast<int>(message.wParam));
            return;
        }
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
#endif
}

void SettingsWindow::update_preview(SettingsPreview preview) {
#ifdef _WIN32
    if(impl_ != nullptr) {
        impl_->preview = std::move(preview);
        if(impl_->hwnd != nullptr) {
            InvalidateRect(impl_->hwnd, nullptr, FALSE);
        }
    }
#else
    static_cast<void>(preview);
#endif
}

}  // namespace aerial_touch

#else

#include <cmath>

namespace aerial_touch {

PreviewZone classify_preview_zone(const std::optional<float> distance_mm,
                                  const float touch_threshold_mm,
                                  const float release_threshold_mm) {
    if(!distance_mm.has_value() || !std::isfinite(*distance_mm)) {
        return PreviewZone::Unknown;
    }
    if(*distance_mm <= touch_threshold_mm) {
        return PreviewZone::Touch;
    }
    if(*distance_mm >= release_threshold_mm) {
        return PreviewZone::Release;
    }
    return PreviewZone::Hold;
}

SettingsWindow::SettingsWindow() = default;
SettingsWindow::~SettingsWindow() = default;

bool SettingsWindow::create(const AppConfig&, std::filesystem::path, ApplyCallback) {
    return false;
}
void SettingsWindow::show() {}
void SettingsWindow::hide() {}
bool SettingsWindow::visible() const { return false; }
void SettingsWindow::process_messages() {}
void SettingsWindow::update_preview(SettingsPreview) {}

}  // namespace aerial_touch

#endif
