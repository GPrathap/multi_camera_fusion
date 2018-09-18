load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "parking_gflags",
    srcs = [
        "parking_gflags.cc",
    ],
    hdrs = [
        "parking_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "common",
    deps = [
        ":parking_gflags",
    ],
)

cpplint()
