load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_binary(
    name = "mediapipe_hand_bridge.dll",
    srcs = [
        "bridge.cc",
        "hand_bridge_api.h",
    ],
    defines = ["MEDIAPIPE_HAND_BRIDGE_EXPORTS"],
    copts = ["/utf-8"],
    linkshared = True,
    deps = [
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_format_cc_proto",
        "//mediapipe/tasks/cc/vision/core:running_mode",
        "//mediapipe/tasks/cc/vision/hand_landmarker:hand_landmarker",
    ],
)
