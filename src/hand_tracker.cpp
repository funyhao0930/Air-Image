#include "aerial_touch/hand_tracker.hpp"

#include "../mediapipe_bridge/hand_bridge_api.h"

#include <array>
#include <utility>

#if defined(_WIN32)
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#endif

namespace aerial_touch {

struct HandTracker::Impl {
#if defined(_WIN32)
    using CreateFn = void* (*)(const char*, char*, int);
    using DetectFn = int (*)(void*, const unsigned char*, int, int, int, std::int64_t, mphb_result*, char*, int);
    using DestroyFn = void (*)(void*);

    HMODULE module{ nullptr };
    CreateFn create{ nullptr };
    DetectFn detect{ nullptr };
    DestroyFn destroy{ nullptr };
    void* handle{ nullptr };
#endif
    std::string error;
};

HandTracker::HandTracker(const std::filesystem::path& dll_path, const std::filesystem::path& model_path)
    : impl_(std::make_unique<Impl>()) {
#if defined(_WIN32)
    impl_->module = LoadLibraryW(dll_path.c_str());
    if(impl_->module == nullptr) {
        impl_->error = "MediaPipe hand bridge DLL could not be loaded: " + dll_path.string();
        return;
    }

    impl_->create  = reinterpret_cast<Impl::CreateFn>(GetProcAddress(impl_->module, "mphb_create"));
    impl_->detect  = reinterpret_cast<Impl::DetectFn>(GetProcAddress(impl_->module, "mphb_detect_rgb"));
    impl_->destroy = reinterpret_cast<Impl::DestroyFn>(GetProcAddress(impl_->module, "mphb_destroy"));
    if(impl_->create == nullptr || impl_->detect == nullptr || impl_->destroy == nullptr) {
        impl_->error = "MediaPipe hand bridge DLL is missing required exports";
        FreeLibrary(impl_->module);
        impl_->module = nullptr;
        return;
    }

    std::array<char, 512> error_buffer{};
    const std::string model_utf8 = model_path.string();
    impl_->handle = impl_->create(model_utf8.c_str(), error_buffer.data(), static_cast<int>(error_buffer.size()));
    if(impl_->handle == nullptr) {
        impl_->error = error_buffer.data();
        if(impl_->error.empty()) {
            impl_->error = "MediaPipe hand bridge could not create a tracker";
        }
    }
#else
    (void)dll_path;
    (void)model_path;
    impl_->error = "MediaPipe hand bridge dynamic loading is only implemented on Windows";
#endif
}

HandTracker::~HandTracker() {
#if defined(_WIN32)
    if(impl_ && impl_->handle != nullptr && impl_->destroy != nullptr) {
        impl_->destroy(impl_->handle);
    }
    if(impl_ && impl_->module != nullptr) {
        FreeLibrary(impl_->module);
    }
#endif
}

HandTracker::HandTracker(HandTracker&&) noexcept = default;
HandTracker& HandTracker::operator=(HandTracker&&) noexcept = default;

bool HandTracker::available() const {
#if defined(_WIN32)
    return impl_ && impl_->handle != nullptr;
#else
    return false;
#endif
}

const std::string& HandTracker::error() const {
    return impl_->error;
}

HandObservation HandTracker::detect_rgb(const unsigned char* rgb,
                                        const int width,
                                        const int height,
                                        const int stride_bytes,
                                        const std::int64_t timestamp_ms) {
    HandObservation observation;
#if defined(_WIN32)
    if(!available() || rgb == nullptr || width <= 0 || height <= 0 || stride_bytes < width * 3) {
        return observation;
    }

    mphb_result result{};
    std::array<char, 512> error_buffer{};
    if(impl_->detect(impl_->handle, rgb, width, height, stride_bytes, timestamp_ms, &result, error_buffer.data(),
                     static_cast<int>(error_buffer.size())) == 0) {
        impl_->error = error_buffer.data();
        return observation;
    }

    observation.detected       = result.detected != 0;
    observation.landmark_count = result.landmark_count;
    for(int index = 0; index < result.landmark_count && index < static_cast<int>(observation.landmarks.size()); ++index) {
        observation.landmarks[static_cast<std::size_t>(index)] = {
            result.landmarks[index].x,
            result.landmarks[index].y,
            result.landmarks[index].z,
        };
    }
#else
    (void)rgb;
    (void)width;
    (void)height;
    (void)stride_bytes;
    (void)timestamp_ms;
#endif
    return observation;
}

}  // namespace aerial_touch
