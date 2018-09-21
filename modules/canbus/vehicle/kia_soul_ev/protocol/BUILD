load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_kia_soul_ev_protocol",
    srcs = [
        "brake_72.cc",
        "brake_73.cc",
        "brake_enable.cc",
        "brake_disable.cc",    
        "steering_83.cc",
        "steering_84.cc",
        "steering_2B0.cc",        
        "steering_enable.cc",
        "steering_disable.cc",
        "throttle_92.cc",
        "throttle_93.cc",
        "throttle_enable.cc",
        "throttle_disable.cc",
        "wheels_4B0.cc",
        "gear_200.cc",
    ],
    hdrs = [
        "brake_72.h",
        "brake_73.h",
        "brake_enable.h",
        "brake_disable.h",        
        "steering_83.h",
        "steering_84.h",
        "steering_2B0.h",
        "steering_enable.h",
        "steering_disable.h",
        "throttle_92.h",
        "throttle_93.h",
        "oscc_magic.h",
        "throttle_enable.h",
        "throttle_disable.h",
        "wheels_4B0.h",
        "gear_200.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)
cpplint()
