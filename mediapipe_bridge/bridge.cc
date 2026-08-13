#define MEDIAPIPE_HAND_BRIDGE_EXPORTS
#include "aerial_touch_bridge/hand_bridge_api.h"

#include "mediapipe/framework/formats/image.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/tasks/cc/vision/core/running_mode.h"
#include "mediapipe/tasks/cc/vision/hand_landmarker/hand_landmarker.h"

#include <algorithm>
#include <cstring>
#include <exception>
#include <memory>
#include <string>

namespace {

using HandLandmarker = mediapipe::tasks::vision::hand_landmarker::HandLandmarker;
using HandLandmarkerOptions = mediapipe::tasks::vision::hand_landmarker::HandLandmarkerOptions;

struct Bridge {
    std::unique_ptr<HandLandmarker> landmarker;
};

void write_error(const std::string& message, char* destination, const int capacity) {
    if(destination == nullptr || capacity <= 0) {
        return;
    }
    const auto count = std::min(message.size(), static_cast<std::size_t>(capacity - 1));
    std::memcpy(destination, message.data(), count);
    destination[count] = '\0';
}

void clear_error(char* destination, const int capacity) {
    if(destination != nullptr && capacity > 0) {
        destination[0] = '\0';
    }
}

}  // namespace

extern "C" MPHB_API void* mphb_create(const char* model_path, char* error_message, const int error_capacity) {
    clear_error(error_message, error_capacity);
    if(model_path == nullptr || model_path[0] == '\0') {
        write_error("Model path is empty", error_message, error_capacity);
        return nullptr;
    }
    try {
        auto options = std::make_unique<HandLandmarkerOptions>();
        options->base_options.model_asset_path = model_path;
        options->running_mode = mediapipe::tasks::vision::core::RunningMode::VIDEO;
        options->num_hands = 1;

        auto landmarker = HandLandmarker::Create(std::move(options));
        if(!landmarker.ok()) {
            write_error(landmarker.status().ToString(), error_message, error_capacity);
            return nullptr;
        }
        return new Bridge{ std::move(landmarker.value()) };
    }
    catch(const std::exception& error) {
        write_error(error.what(), error_message, error_capacity);
        return nullptr;
    }
    catch(...) {
        write_error("Unknown exception while creating Hand Landmarker", error_message, error_capacity);
        return nullptr;
    }
}

extern "C" MPHB_API int mphb_detect_rgb(void* handle,
                                         const unsigned char* rgb,
                                         const int width,
                                         const int height,
                                         const int stride_bytes,
                                         const int64_t timestamp_ms,
                                         mphb_result* result,
                                         char* error_message,
                                         const int error_capacity) {
    clear_error(error_message, error_capacity);
    if(result != nullptr) {
        *result = {};
    }
    if(handle == nullptr || rgb == nullptr || result == nullptr || width <= 0 || height <= 0 || stride_bytes < width * 3) {
        write_error("Invalid detect_rgb argument", error_message, error_capacity);
        return 0;
    }
    try {
        auto image_frame = std::make_shared<mediapipe::ImageFrame>();
        image_frame->CopyPixelData(mediapipe::ImageFormat::SRGB, width, height, stride_bytes, rgb, 1);
        mediapipe::Image image(image_frame);

        auto detection = static_cast<Bridge*>(handle)->landmarker->DetectForVideo(std::move(image), timestamp_ms);
        if(!detection.ok()) {
            write_error(detection.status().ToString(), error_message, error_capacity);
            return 0;
        }
        if(detection->hand_landmarks.empty()) {
            return 1;
        }

        const auto& landmarks = detection->hand_landmarks.front().landmarks;
        const int count = std::min<int>(21, static_cast<int>(landmarks.size()));
        result->detected = count == 21 ? 1 : 0;
        result->landmark_count = count;
        for(int index = 0; index < count; ++index) {
            result->landmarks[index] = { landmarks[index].x, landmarks[index].y, landmarks[index].z };
        }
        return 1;
    }
    catch(const std::exception& error) {
        write_error(error.what(), error_message, error_capacity);
        return 0;
    }
    catch(...) {
        write_error("Unknown exception during Hand Landmarker detection", error_message, error_capacity);
        return 0;
    }
}

extern "C" MPHB_API void mphb_destroy(void* handle) {
    delete static_cast<Bridge*>(handle);
}
