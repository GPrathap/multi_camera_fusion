load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "sim_bridge_config_proto",
    deps = [
        ":sim_bridge_config_proto_lib",
    ],
)

proto_library(
    name = "sim_bridge_config_proto_lib",
    srcs = ["sim_bridge_config.proto"],
    deps = [
        "//modules/common/proto:common_proto_lib",
    ],
)


cpplint()
