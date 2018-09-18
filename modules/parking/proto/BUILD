package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "parking_proto",
    deps = [
        ":parking_proto_lib",
    ],
)

proto_library(
    name = "parking_proto_lib",
    srcs = [
        "parking_status.proto",
        "parking_conf.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)
