load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kia_soul_ev_vehicle_factory",
    srcs = [
        "kia_soul_ev_vehicle_factory.cc",
    ],
    hdrs = [
        "kia_soul_ev_vehicle_factory.h",
    ],
    deps = [
        ":kia_soul_ev_controller",
        ":kia_soul_ev_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "kia_soul_ev_message_manager",
    srcs = [
        "kia_soul_ev_message_manager.cc",
    ],
    hdrs = [
        "kia_soul_ev_message_manager.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle/kia_soul_ev/protocol:canbus_kia_soul_ev_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "kia_soul_ev_controller",
    srcs = [
        "kia_soul_ev_controller.cc",
    ],
    hdrs = [
        "kia_soul_ev_controller.h",
    ],
    deps = [
        ":kia_soul_ev_message_manager",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/kia_soul_ev/protocol:canbus_kia_soul_ev_protocol",
        "//modules/common/kv_db",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@gtest//:gtest",
    ],
)

cc_test(
    name = "kia_soul_ev_vehicle_factory_test",
    size = "small",
    srcs = ["kia_soul_ev_vehicle_factory_test.cc"],
    deps = [
        ":kia_soul_ev_vehicle_factory",
        "@gtest//:main",
    ],
)

cpplint()
