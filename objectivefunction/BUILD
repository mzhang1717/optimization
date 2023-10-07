cc_library(
    name = "function_base",
    hdrs = ["costfunctionbase.h"],
    visibility = ["//objectivefunction/test:__pkg__",
                 "//algorithms:__pkg__",],
)

cc_library(
    name = "function_example",
    srcs = ["costfunctionexample.cpp"],
    hdrs = ["costfunctionexample.h"],
    deps =[":function_base",],
    visibility = ["//objectivefunction/test:__pkg__"],
)

cc_library(
    name = "function_quadratic",
    hdrs = ["costfunctionquadratic.h"],
    deps =[":function_base",],
    visibility = ["//objectivefunction/test:__pkg__"],
)

cc_library(
    name = "function_himmelblau",
    srcs = ["costfunctionhimmelblau.cpp"],
    hdrs = ["costfunctionhimmelblau.h"],
    deps =[":function_base",],
    visibility = ["//objectivefunction/test:__pkg__",
    "//algorithms/test:__pkg__",],
)