#include "aerial_touch/settings_window.hpp"

#include <algorithm>
#include <cmath>

namespace aerial_touch {

SettingsLayout calculate_settings_layout(const int client_width, const int client_height) {
    constexpr int kMargin = 20;
    constexpr int kColumnGap = 15;
    constexpr int kPanelTop = 75;
    constexpr int kPanelGap = 15;
    constexpr int kBottomReserve = 105;
    constexpr int kMinimumPanelHeight = 300;

    const int width = std::max(900, client_width);
    const int height = std::max(780, client_height);
    const int panel_bottom = std::max(kPanelTop + kMinimumPanelHeight * 2 + kPanelGap,
                                      height - kBottomReserve);
    const int available_panel_height = panel_bottom - kPanelTop - kPanelGap;
    const int top_panel_height = std::max(kMinimumPanelHeight, available_panel_height / 2);
    const int bottom_panel_top = kPanelTop + top_panel_height + kPanelGap;
    const int column_width = (width - kMargin * 2 - kColumnGap) / 2;
    const int right_column_left = kMargin + column_width + kColumnGap;

    SettingsLayout layout;
    layout.touch_group = { kMargin, kPanelTop, kMargin + column_width, kPanelTop + top_panel_height };
    layout.depth_group = { kMargin, bottom_panel_top, kMargin + column_width, panel_bottom };
    layout.preview_group = { right_column_left, kPanelTop, width - kMargin, kPanelTop + top_panel_height };
    layout.keypad_group = { right_column_left, bottom_panel_top, width - kMargin, panel_bottom };
    layout.preview_rect = { right_column_left + 15, kPanelTop + 54, width - kMargin - 15, kPanelTop + top_panel_height - 15 };
    layout.status_y = height - 88;
    layout.path_y = height - 62;
    layout.buttons_y = height - 42;
    return layout;
}

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

}  // namespace aerial_touch

#ifdef _WIN32

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>
#include <commctrl.h>

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

constexpr int kFieldCount = 6;
constexpr int kCameraFieldCount = 3;
constexpr int kFirstEditId = 1000;
constexpr int kFirstSliderId = 1100;
constexpr int kRestoreButtonId = 1201;
constexpr int kCancelButtonId = 1202;
constexpr int kApplyButtonId = 1203;
constexpr int kStatusId = 1300;
constexpr int kPathId = 1301;
constexpr int kFirstErrorId = 1400;
constexpr int kFirstCameraControlId = 1500;

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
        { L"校正點最小距離\r\n避免基準點太接近", L"mm", 20.0, 300.0, 10.0, false },
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
    default:
        break;
    }
}

bool configs_equal(const AppConfig& left, const AppConfig& right) {
    return left.camera.depth_work_mode == right.camera.depth_work_mode
           && left.camera.depth_precision == right.camera.depth_precision
           && left.camera.preferred_fps == right.camera.preferred_fps
           && left.depth.sample_radius == right.depth.sample_radius
           && left.touch.touch_threshold_mm == right.touch.touch_threshold_mm
           && left.touch.release_threshold_mm == right.touch.release_threshold_mm
           && left.touch.min_approach_velocity_mm_s == right.touch.min_approach_velocity_mm_s
           && left.touch.tracking_timeout_ms == right.touch.tracking_timeout_ms
           && left.keypad.boundary_hysteresis_mm == right.keypad.boundary_hysteresis_mm
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

struct SettingsWindow::Impl {
    HWND hwnd{};
    AppConfig applied_config{};
    CameraCapabilities camera_capabilities{};
    std::filesystem::path config_path;
    ApplyCallback apply_callback;
    SettingsPreview preview{};
    SettingsLayout layout{};
    std::array<HWND, kFieldCount> labels{};
    std::array<HWND, kFieldCount> edits{};
    std::array<HWND, kFieldCount> sliders{};
    std::array<HWND, kFieldCount> units{};
    std::array<HWND, kFieldCount> errors{};
    std::array<HWND, kCameraFieldCount> camera_labels{};
    std::array<HWND, kCameraFieldCount> camera_controls{};
    std::vector<std::string> camera_work_mode_values;
    HWND status{};
    HWND path_label{};
    HWND restore_button{};
    HWND cancel_button{};
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

    void create_field(const int index) {
        const auto& spec = field_specs()[static_cast<std::size_t>(index)];
        const DWORD label_style = WS_CHILD | WS_VISIBLE | SS_LEFT;
        labels[static_cast<std::size_t>(index)] = create_control(L"STATIC", spec.label, label_style, 0, 0, 0, 0, 0);

        const int slider_id = kFirstSliderId + index;
        sliders[static_cast<std::size_t>(index)] = create_control(TRACKBAR_CLASSW, L"", WS_CHILD | WS_VISIBLE | WS_TABSTOP,
                                                                    0, 0, 0, 0, slider_id);
        SendMessageW(sliders[static_cast<std::size_t>(index)], TBM_SETRANGE, TRUE,
                     MAKELONG(0, static_cast<short>(std::lround(spec.slider_max * spec.slider_scale))));
        SendMessageW(sliders[static_cast<std::size_t>(index)], TBM_SETPAGESIZE, 0, 1);

        edits[static_cast<std::size_t>(index)] = create_control(
            L"EDIT", L"", WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL | ES_RIGHT,
            0, 0, 0, 0, kFirstEditId + index);
        SendMessageW(edits[static_cast<std::size_t>(index)], EM_SETLIMITTEXT, 32, 0);
        units[static_cast<std::size_t>(index)] = create_control(L"STATIC", spec.unit, label_style, 0, 0, 0, 0, 0);
        errors[static_cast<std::size_t>(index)] = create_control(L"STATIC", L"", label_style, 0, 0, 0, 0,
                                                                  kFirstErrorId + index);
        if(errors[static_cast<std::size_t>(index)] != nullptr) {
            set_font(errors[static_cast<std::size_t>(index)], small_font);
            ShowWindow(errors[static_cast<std::size_t>(index)], SW_HIDE);
        }
    }

    void create_camera_controls() {
        constexpr std::array<const wchar_t*, kCameraFieldCount> labels{ L"深度工作模式", L"深度精度", L"相機 FPS" };
        for(int index = 0; index < kCameraFieldCount; ++index) {
            camera_labels[static_cast<std::size_t>(index)] =
                create_control(L"STATIC", labels[static_cast<std::size_t>(index)], WS_CHILD | WS_VISIBLE | SS_LEFT,
                               0, 0, 0, 0, 0);
        }

        const DWORD combo_style = WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_VSCROLL | CBS_DROPDOWNLIST;
        camera_controls[0] = create_control(L"COMBOBOX", L"", combo_style, 0, 0, 0, 0, kFirstCameraControlId);
        camera_controls[1] = create_control(L"COMBOBOX", L"", combo_style, 0, 0, 0, 0, kFirstCameraControlId + 1);
        camera_controls[2] = create_control(L"COMBOBOX", L"", combo_style, 0, 0, 0, 0, kFirstCameraControlId + 2);

        camera_work_mode_values.push_back({});
        std::wstring current_mode = L"保持目前模式";
        if(!camera_capabilities.current_depth_work_mode.empty()) {
            current_mode += L"（" + utf8_to_wide(camera_capabilities.current_depth_work_mode) + L"）";
        }
        SendMessageW(camera_controls[0], CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(current_mode.c_str()));
        for(const auto& value : camera_capabilities.depth_work_modes) {
            if(value.empty()) {
                continue;
            }
            camera_work_mode_values.push_back(value);
            const auto wide = utf8_to_wide(value);
            SendMessageW(camera_controls[0], CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(wide.c_str()));
        }

        for(const auto& value : camera_capabilities.depth_precisions) {
            const auto wide = utf8_to_wide(value);
            SendMessageW(camera_controls[1], CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(wide.c_str()));
        }
        for(const int value : camera_capabilities.fps_values) {
            const auto wide = std::to_wstring(value);
            SendMessageW(camera_controls[2], CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(wide.c_str()));
        }
        EnableWindow(camera_controls[1], camera_capabilities.depth_precisions.empty() ? FALSE : TRUE);
        EnableWindow(camera_controls[2], camera_capabilities.fps_values.empty() ? FALSE : TRUE);
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

        path_label = create_control(L"STATIC", L"", WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 0, 0, 0, kPathId);
        set_font(path_label, small_font);
        status = create_control(L"STATIC", L"", WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 0, 0, 0, kStatusId);
        set_font(status, small_font);

        create_camera_controls();

        for(int index = 0; index < kFieldCount; ++index) {
            create_field(index);
        }

        restore_button = create_control(L"BUTTON", L"恢復預設值", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
                                        0, 0, 0, 0, kRestoreButtonId);
        cancel_button = create_control(L"BUTTON", L"取消", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
                                       0, 0, 0, 0, kCancelButtonId);
        apply_button = create_control(L"BUTTON", L"套用並儲存", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_DEFPUSHBUTTON,
                                      0, 0, 0, 0, kApplyButtonId);
        set_font(restore_button, font);
        set_font(cancel_button, font);
        set_font(apply_button, font);
        layout_controls();
        load_config_to_controls(applied_config);
        const std::wstring path_text = std::wstring(L"設定檔：") + config_path.wstring();
        set_window_text(path_label, path_text);
    }

    static RECT to_rect(const SettingsRect& rect) {
        return { rect.left, rect.top, rect.right, rect.bottom };
    }

    static void move_control(const HWND control, const int x, const int y, const int width, const int height) {
        if(control != nullptr) {
            SetWindowPos(control, nullptr, x, y, std::max(0, width), std::max(0, height),
                         SWP_NOZORDER | SWP_NOACTIVATE);
        }
    }

    void layout_field(const int index, const SettingsRect& group, const int y, const int row_height) {
        const int inner_left = group.left + 15;
        const int inner_right = group.right - 15;
        const int available = std::max(240, inner_right - inner_left);
        const int label_width = std::min(140, std::max(112, available / 3));
        const int unit_width = 32;
        const int edit_width = 62;
        const int slider_left = inner_left + label_width + 7;
        const int edit_left = inner_right - unit_width - 5 - edit_width;
        const int slider_width = std::max(90, edit_left - slider_left - 7);
        const int error_width = std::max(90, inner_right - slider_left);

        move_control(labels[static_cast<std::size_t>(index)], inner_left, y, label_width, 38);
        move_control(sliders[static_cast<std::size_t>(index)], slider_left, y + 1, slider_width, 30);
        move_control(edits[static_cast<std::size_t>(index)], edit_left, y, edit_width, 25);
        move_control(units[static_cast<std::size_t>(index)], inner_right - unit_width, y + 5, unit_width, 20);
        move_control(errors[static_cast<std::size_t>(index)], slider_left, y + 35, error_width, std::max(18, row_height - 37));
    }

    void layout_camera_field(const int index, const SettingsRect& group, const int y) {
        const int inner_left = group.left + 15;
        const int inner_right = group.right - 15;
        const int label_width = 130;
        move_control(camera_labels[static_cast<std::size_t>(index)], inner_left, y + 4, label_width, 22);
        move_control(camera_controls[static_cast<std::size_t>(index)], inner_left + label_width + 7, y,
                     std::max(100, inner_right - inner_left - label_width - 7), 150);
    }

    void layout_controls() {
        RECT client{};
        GetClientRect(hwnd, &client);
        layout = calculate_settings_layout(client.right - client.left, client.bottom - client.top);

        const int touch_row_height = std::max(58, (layout.touch_group.bottom - layout.touch_group.top - 62) / 4);
        for(int index = 0; index < 4; ++index) {
            layout_field(index, layout.touch_group, layout.touch_group.top + 53 + index * touch_row_height, touch_row_height);
        }
        constexpr int camera_row_height = 34;
        const int camera_top = layout.depth_group.top + 50;
        for(int index = 0; index < kCameraFieldCount; ++index) {
            layout_camera_field(index, layout.depth_group, camera_top + index * camera_row_height);
        }
        const int depth_fields_top = camera_top + kCameraFieldCount * camera_row_height + 4;
        const int depth_row_height = std::max(58, (layout.depth_group.bottom - depth_fields_top - 10) / 2);
        layout_field(4, layout.depth_group, depth_fields_top, depth_row_height);
        layout_field(5, layout.depth_group, depth_fields_top + depth_row_height, depth_row_height);

        const int button_gap = 10;
        const int apply_width = 120;
        const int cancel_width = 75;
        const int restore_width = 105;
        const int apply_left = layout.preview_group.right - 20 - apply_width;
        const int cancel_left = apply_left - button_gap - cancel_width;
        const int restore_left = cancel_left - button_gap - restore_width;
        move_control(restore_button, restore_left, layout.buttons_y, restore_width, 32);
        move_control(cancel_button, cancel_left, layout.buttons_y, cancel_width, 32);
        move_control(apply_button, apply_left, layout.buttons_y, apply_width, 32);
        move_control(status, 24, layout.status_y, std::max(200, restore_left - 39), 22);
        move_control(path_label, 24, layout.path_y, std::max(200, restore_left - 39), 18);
        InvalidateRect(hwnd, nullptr, FALSE);
    }

    void set_field_error(const int index, const std::wstring& error) {
        HWND error_control = errors[static_cast<std::size_t>(index)];
        set_window_text(error_control, error);
        ShowWindow(error_control, error.empty() ? SW_HIDE : SW_SHOW);
    }

    void set_status(const std::wstring& message, const COLORREF color) {
        set_window_text(status, message);
        InvalidateRect(status, nullptr, FALSE);
        status_color = color;
    }

    COLORREF status_color{ kMuted };

    static bool select_combo_value(const HWND combo, const std::wstring& value) {
        const LRESULT index = SendMessageW(combo, CB_FINDSTRINGEXACT, static_cast<WPARAM>(-1),
                                           reinterpret_cast<LPARAM>(value.c_str()));
        if(index == CB_ERR) {
            return false;
        }
        SendMessageW(combo, CB_SETCURSEL, static_cast<WPARAM>(index), 0);
        return true;
    }

    void load_config_to_controls(const AppConfig& config) {
        syncing = true;
        int mode_index = 0;
        if(!config.camera.depth_work_mode.empty()) {
            const auto found = std::find(camera_work_mode_values.begin(), camera_work_mode_values.end(),
                                         config.camera.depth_work_mode);
            if(found != camera_work_mode_values.end()) {
                mode_index = static_cast<int>(std::distance(camera_work_mode_values.begin(), found));
            }
        }
        SendMessageW(camera_controls[0], CB_SETCURSEL, static_cast<WPARAM>(mode_index), 0);

        if(!camera_capabilities.depth_precisions.empty()
           && !select_combo_value(camera_controls[1], utf8_to_wide(config.camera.depth_precision))) {
            if(camera_capabilities.current_depth_precision.empty()
               || !select_combo_value(camera_controls[1], utf8_to_wide(camera_capabilities.current_depth_precision))) {
                SendMessageW(camera_controls[1], CB_SETCURSEL, 0, 0);
            }
        }
        if(!camera_capabilities.fps_values.empty()
           && !select_combo_value(camera_controls[2], std::to_wstring(config.camera.preferred_fps))) {
            if(camera_capabilities.current_fps <= 0
               || !select_combo_value(camera_controls[2], std::to_wstring(camera_capabilities.current_fps))) {
                SendMessageW(camera_controls[2], CB_SETCURSEL, 0, 0);
            }
        }
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
        const LRESULT mode_index = SendMessageW(camera_controls[0], CB_GETCURSEL, 0, 0);
        if(mode_index == CB_ERR || mode_index < 0
           || static_cast<std::size_t>(mode_index) >= camera_work_mode_values.size()) {
            return false;
        }
        candidate.camera.depth_work_mode = camera_work_mode_values[static_cast<std::size_t>(mode_index)];

        if(!camera_capabilities.depth_precisions.empty()) {
            const LRESULT precision_index = SendMessageW(camera_controls[1], CB_GETCURSEL, 0, 0);
            if(precision_index == CB_ERR || precision_index < 0
               || static_cast<std::size_t>(precision_index) >= camera_capabilities.depth_precisions.size()) {
                return false;
            }
            candidate.camera.depth_precision = camera_capabilities.depth_precisions[static_cast<std::size_t>(precision_index)];
        }
        if(!camera_capabilities.fps_values.empty()) {
            const LRESULT fps_index = SendMessageW(camera_controls[2], CB_GETCURSEL, 0, 0);
            if(fps_index == CB_ERR || fps_index < 0
               || static_cast<std::size_t>(fps_index) >= camera_capabilities.fps_values.size()) {
                return false;
            }
            candidate.camera.preferred_fps = camera_capabilities.fps_values[static_cast<std::size_t>(fps_index)];
        }
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
        InvalidateRect(status, nullptr, FALSE);
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
        const bool camera_changed = candidate.camera.depth_work_mode != applied_config.camera.depth_work_mode
                                    || candidate.camera.depth_precision != applied_config.camera.depth_precision
                                    || candidate.camera.preferred_fps != applied_config.camera.preferred_fps;
        std::string error;
        if(!apply_callback || !apply_callback(candidate, error)) {
            set_status(error.empty() ? L"套用設定失敗" : utf8_to_wide(error), kRed);
            return;
        }
        applied_config = candidate;
        refresh_validation();
        set_status(camera_changed ? L"已儲存；相機設定重新啟動後生效" : L"已套用並儲存", kCyan);
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
        const RECT preview_rect = to_rect(layout.preview_rect);
        HBRUSH brush = CreateSolidBrush(RGB(8, 21, 24));
        FillRect(dc, &preview_rect, brush);
        DeleteObject(brush);

        const int preview_width = static_cast<int>(preview_rect.right - preview_rect.left);
        const int preview_height = static_cast<int>(preview_rect.bottom - preview_rect.top);
        const int plane_left = preview_rect.left + std::max(25, preview_width / 8);
        const int plane_right = preview_rect.right - std::max(25, preview_width / 8);
        const int plane_top = preview_rect.top + std::max(25, preview_height / 7);
        const int plane_bottom = preview_rect.bottom - std::max(65, preview_height / 4);
        const int center_x = (plane_left + plane_right) / 2;
        HPEN plane_pen = CreatePen(PS_SOLID, 1, RGB(75, 143, 139));
        HGDIOBJ old_pen = SelectObject(dc, plane_pen);
        POINT plane[4]{ { plane_left, plane_top + 25 }, { plane_right, plane_top },
                        { plane_right, plane_bottom - 15 }, { plane_left, plane_bottom + 15 } };
        Polygon(dc, plane, 4);
        SelectObject(dc, old_pen);
        DeleteObject(plane_pen);

        const PreviewZone zone = classify_preview_zone(preview.distance_mm, applied_config.touch.touch_threshold_mm,
                                                        applied_config.touch.release_threshold_mm);
        const COLORREF zone_color = zone == PreviewZone::Touch ? kRed
                                     : zone == PreviewZone::Release ? kCyan
                                     : zone == PreviewZone::Hold ? kAmber
                                                                  : kMuted;
        int finger_y = preview_rect.top + 18;
        if(preview.distance_mm.has_value()) {
            const double normalized = std::clamp(static_cast<double>(*preview.distance_mm) / 80.0, 0.0, 1.0);
            const int available_finger_travel = static_cast<int>(plane_bottom - preview_rect.top - 50);
            finger_y = static_cast<int>(preview_rect.top) + 25
                       + static_cast<int>(normalized * std::max(40, available_finger_travel));
        }
        HBRUSH finger_brush = CreateSolidBrush(zone_color);
        HGDIOBJ old_brush = SelectObject(dc, finger_brush);
        Ellipse(dc, center_x - 8, finger_y, center_x + 8, finger_y + 16);
        SelectObject(dc, old_brush);
        DeleteObject(finger_brush);

        HPEN distance_pen = CreatePen(PS_DASH, 1, kAmber);
        old_pen = SelectObject(dc, distance_pen);
        MoveToEx(dc, center_x, finger_y + 16, nullptr);
        LineTo(dc, center_x, plane_bottom);
        SelectObject(dc, old_pen);
        DeleteObject(distance_pen);

        SetBkMode(dc, TRANSPARENT);
        SetTextColor(dc, kMuted);
        SelectObject(dc, small_font);
        const wchar_t* plane_label = L"虛擬鍵盤平面";
        TextOutW(dc, plane_left + 15, plane_bottom + 20, plane_label, static_cast<int>(wcslen(plane_label)));

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
        TextOutW(dc, preview_rect.left + 20, preview_rect.top + 18, distance.c_str(), static_cast<int>(distance.size()));

        SetTextColor(dc, kText);
        SelectObject(dc, small_font);
        const std::wstring tracking = preview.tracking_detected ? L"追蹤：已偵測" : L"追蹤：未偵測";
        const std::wstring key = std::wstring(L"按鍵：")
                                 + (preview.key.has_value() ? utf8_to_wide(*preview.key) : L"-");
        TextOutW(dc, preview_rect.left + 20, preview_rect.bottom - 55, tracking.c_str(), static_cast<int>(tracking.size()));
        TextOutW(dc, preview_rect.left + 140, preview_rect.bottom - 55, key.c_str(), static_cast<int>(key.size()));

        const std::wstring touch = preview.armed ? L"觸控：可觸發" : L"觸控：等待手指離開";
        SetTextColor(dc, preview.armed ? kCyan : kAmber);
        TextOutW(dc, preview_rect.left + 20, preview_rect.bottom - 30, touch.c_str(), static_cast<int>(touch.size()));
        SetTextColor(dc, kMuted);
        const std::wstring hint = std::wstring(L"觸控 ")
                                  + format_value(applied_config.touch.touch_threshold_mm, false)
                                  + L" mm   /   離開 " + format_value(applied_config.touch.release_threshold_mm, false) + L" mm";
        TextOutW(dc, preview_rect.left + 170, preview_rect.bottom - 30, hint.c_str(), static_cast<int>(hint.size()));
    }

    void paint() {
        PAINTSTRUCT paint_struct{};
        const HDC dc = BeginPaint(hwnd, &paint_struct);
        RECT client{};
        GetClientRect(hwnd, &client);
        const HDC buffer = CreateCompatibleDC(dc);
        const HBITMAP bitmap = CreateCompatibleBitmap(dc, client.right, client.bottom);
        HGDIOBJ old_bitmap = nullptr;
        if(buffer != nullptr && bitmap != nullptr) {
            old_bitmap = SelectObject(buffer, bitmap);
        }
        const HDC canvas = old_bitmap != nullptr ? buffer : dc;
        FillRect(canvas, &client, background_brush);
        SetBkMode(canvas, TRANSPARENT);
        SetTextColor(canvas, kCyan);
        SelectObject(canvas, small_font);
        const wchar_t* eyebrow = L"AERIAL TOUCH / TUNING CONSOLE";
        TextOutW(canvas, 24, 18, eyebrow, static_cast<int>(wcslen(eyebrow)));
        SetTextColor(canvas, kText);
        SelectObject(canvas, heading_font);
        const wchar_t* title = L"參數設定";
        TextOutW(canvas, 24, 38, title, static_cast<int>(wcslen(title)));
        SetTextColor(canvas, kMuted);
        SelectObject(canvas, small_font);
        const wchar_t* description = L"互動參數可即時套用；相機參數重新啟動後生效。";
        TextOutW(canvas, 155, 44, description, static_cast<int>(wcslen(description)));

        draw_group(canvas, to_rect(layout.touch_group), L"觸控判定", L"TOUCH LOGIC");
        draw_group(canvas, to_rect(layout.depth_group), L"深度與校正", L"DEPTH / CALIBRATION");
        draw_group(canvas, to_rect(layout.preview_group), L"觸控距離預覽", L"LIVE MODEL");
        draw_group(canvas, to_rect(layout.keypad_group), L"鍵盤幾何", L"KEYPAD GEOMETRY");
        RECT geometry_hint{ layout.keypad_group.left + 15, layout.keypad_group.top + 58,
                            layout.keypad_group.right - 15, layout.keypad_group.bottom - 15 };
        SetTextColor(canvas, kMuted);
        DrawTextW(canvas, L"鍵盤尺寸由 C 校正的 7 個邊界點自動取得。\r\n完成校正後才會啟用。",
                  -1, &geometry_hint, DT_LEFT | DT_TOP | DT_WORDBREAK);
        draw_preview(canvas);
        if(old_bitmap != nullptr) {
            BitBlt(dc, 0, 0, client.right, client.bottom, canvas, 0, 0, SRCCOPY);
            SelectObject(buffer, old_bitmap);
        }
        if(bitmap != nullptr) {
            DeleteObject(bitmap);
        }
        if(buffer != nullptr) {
            DeleteDC(buffer);
        }
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
        case WM_SIZE:
            if(w_param != SIZE_MINIMIZED) {
                self->layout_controls();
            }
            return 0;
        case WM_GETMINMAXINFO: {
            auto* limits = reinterpret_cast<MINMAXINFO*>(l_param);
            RECT minimum_rect{ 0, 0, 900, 780 };
            const DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_THICKFRAME;
            AdjustWindowRectEx(&minimum_rect, style, FALSE, WS_EX_TOOLWINDOW);
            limits->ptMinTrackSize.x = minimum_rect.right - minimum_rect.left;
            limits->ptMinTrackSize.y = minimum_rect.bottom - minimum_rect.top;
            return 0;
        }
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
            if(id >= kFirstCameraControlId && id < kFirstCameraControlId + kCameraFieldCount
               && notification == CBN_SELCHANGE && !self->syncing) {
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
        const DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_THICKFRAME;
        RECT window_rect{ 0, 0, 920, 780 };
        AdjustWindowRectEx(&window_rect, style, FALSE, WS_EX_TOOLWINDOW);
        const int window_width = window_rect.right - window_rect.left;
        const int window_height = window_rect.bottom - window_rect.top;
        hwnd = CreateWindowExW(WS_EX_TOOLWINDOW, L"AerialTouchSettingsWindow", L"Aerial Touch · 參數設定", style,
                              CW_USEDEFAULT, CW_USEDEFAULT, window_width, window_height, nullptr, nullptr, instance, this);
        if(hwnd == nullptr) {
            return false;
        }
        RECT work_area{};
        SystemParametersInfoW(SPI_GETWORKAREA, 0, &work_area, 0);
        const int x = work_area.left + (work_area.right - work_area.left - window_width) / 2;
        const int y = work_area.top + (work_area.bottom - work_area.top - window_height) / 2;
        SetWindowPos(hwnd, HWND_TOP, x, y, window_width, window_height, SWP_NOACTIVATE);
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

bool SettingsWindow::create(const AppConfig& config, CameraCapabilities camera_capabilities,
                            std::filesystem::path config_path, ApplyCallback apply_callback) {
#ifdef _WIN32
    if(impl_ != nullptr) {
        return impl_->hwnd != nullptr;
    }
    impl_ = new Impl;
    impl_->applied_config = config;
    impl_->camera_capabilities = std::move(camera_capabilities);
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
    static_cast<void>(camera_capabilities);
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
        if(impl_->preview.distance_mm == preview.distance_mm && impl_->preview.tracking_detected == preview.tracking_detected
           && impl_->preview.key == preview.key && impl_->preview.armed == preview.armed) {
            return;
        }
        impl_->preview = std::move(preview);
        if(impl_->hwnd != nullptr) {
            const RECT preview_rect = Impl::to_rect(impl_->layout.preview_rect);
            InvalidateRect(impl_->hwnd, &preview_rect, FALSE);
        }
    }
#else
    static_cast<void>(preview);
#endif
}

}  // namespace aerial_touch

#else

namespace aerial_touch {

SettingsWindow::SettingsWindow() = default;
SettingsWindow::~SettingsWindow() = default;

bool SettingsWindow::create(const AppConfig&, CameraCapabilities, std::filesystem::path, ApplyCallback) {
    return false;
}
void SettingsWindow::show() {}
void SettingsWindow::hide() {}
bool SettingsWindow::visible() const { return false; }
void SettingsWindow::process_messages() {}
void SettingsWindow::update_preview(SettingsPreview) {}

}  // namespace aerial_touch

#endif
