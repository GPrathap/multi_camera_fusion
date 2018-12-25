load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dlf",
    srcs = ["dlf_affinity_tracker.cc"],
    hdrs = ["dlf_affinity_tracker.h"],
    deps = [
        "//modules/perception/obstacle/base",
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/obstacle/camera/tracker:base_affinity",
        "//modules/perception/obstacle/camera/tracker/common:tracker_common",
        "@eigen",
        "@opencv2//:core",
        "@opencv2//:ml",
        "@opencv2//:highgui",
        "@opencv2//:video",
    ],
)

cpplint()
