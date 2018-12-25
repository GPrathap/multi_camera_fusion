load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tracker_common",
    srcs = ["dtw.cc"],
    hdrs = ["dtw.h"],
    deps = [
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/tracker:base_affinity",
    ],
)

cpplint()
