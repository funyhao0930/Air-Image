licenses(["notice"])

package(default_visibility = ["//visibility:public"])

config_setting(
    name = "opt_build",
    values = {"compilation_mode": "opt"},
)

config_setting(
    name = "dbg_build",
    values = {"compilation_mode": "dbg"},
)

cc_library(
    name = "opencv",
    srcs = select({
        ":opt_build": glob(["lib/opencv_*.lib"]) + glob(["bin/opencv_*.dll"]),
        ":dbg_build": glob(["debug/lib/opencv_*d.lib"]) + glob(["debug/bin/opencv_*d.dll"]),
    }),
    hdrs = glob(["include/opencv4/opencv2/**/*.h*"]),
    includes = ["include/opencv4"],
    linkstatic = 1,
)
