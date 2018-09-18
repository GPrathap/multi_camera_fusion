load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "parking_lib",
    srcs = [
        "parking.cc",
    ],
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/parking/common",
        "//modules/parking/proto:parking_proto",
        "//modules/control/proto:control_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "parking",
    srcs = ["main.cc"],
    deps = [
        ":parking_lib",
        "//external:gflags",
    ],
)


cpplint()
