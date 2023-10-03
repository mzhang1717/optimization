cc_library(
    name = "function_example",
    srcs = ["costfunctionexample.cpp"],
    hdrs = ["costfunctionexample.h", "costfunctionbase.h"],
)

cc_library(
    name = "function_quadratic",
    hdrs = ["costfunctionquadratic.h", "costfunctionbase.h"],
)

cc_binary(
    name = "test_function",
    srcs = ["test_function.cpp"],
    deps = [":function_example", "function_quadratic",
    ],
)