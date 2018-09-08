load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sim_bridge_base",
    srcs = [
        "sim_bridge_base.cc",
    ],
    hdrs = [
        "sim_bridge_base.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/configs:config_gflags",
        "//modules/common/status",
        "//modules/sim_bridge/common:sim_bridge_common",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "sim_bridge_lib",
    srcs = [
        "sim_bridge.cc",
    ],
    hdrs = [
        "sim_bridge.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:apollo_app",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/localization/proto:imu_proto",
        "//modules/sim_bridge/proto:sim_bridge_config_proto",
        "//modules/sim_bridge/unity_sim_bridge:unity_sim_bridge",
        "//modules/sim_bridge/ros_localization_bridge:ros_localization_bridge",
        "//modules/sim_bridge:sim_bridge_base",
    ],
)

cc_binary(
    name = "sim_bridge",
    srcs = [
        "main.cc",
    ],
    data = [
        "//modules/sim_bridge/conf:sim_bridge_adapter_manager_config",
    ],
    deps = [
        ":sim_bridge_lib",
    ],
)

cpplint()
