load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ros_localization_bridge",
    srcs = [
        "ros_localization_bridge.cc",
    ],
    hdrs = [
        "ros_localization_bridge.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/sim_bridge:sim_bridge_base",
        "//modules/sim_bridge/proto:sim_bridge_config_proto",
        "//modules/sim_bridge/common:sim_bridge_common",
        "//modules/localization/proto:localization_proto",
        "//modules/common/configs:vehicle_config_helper",
        "@ros//:ros_common",
    ],
)


cpplint()
