load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_kia_soul_ev_protocol",
    srcs = [
        "brake_72.cc",
        "brake_73.cc",
        "steering_83.cc",
        "steering_84.cc",
        "throttle_92.cc",
        "throttle_93.cc",
    ],
    hdrs = [
        "brake_72.h",
        "brake_73.h",
        "steering_83.h",
        "steering_84.h",
        "throttle_92.h",
        "throttle_93.h",
        "oscc_magic.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)
cpplint()
