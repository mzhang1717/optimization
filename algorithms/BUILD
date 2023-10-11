cc_library(
    name = "optimizer_base",
    srcs = ["optimizerbase.cpp"],
    hdrs = ["optimizerbase.h"],
    deps = ["//objectivefunction:function_base",],
    visibility = ["//algorithms/test:__pkg__"],
)

cc_library(
    name = "optimizer_descent",
    srcs = ["optimizerdescent.cpp"],
    hdrs = ["optimizerdescent.h"],
    deps = ["//objectivefunction:function_base",
    ":optimizer_base",],
    visibility = ["//algorithms/test:__pkg__"],
)

cc_library(
    name = "optimizer_newton",
    srcs = ["optimizernewton.cpp"],
    hdrs = ["optimizernewton.h"],
    deps = ["//objectivefunction:function_base",
    ":optimizer_base",],
    visibility = ["//algorithms/test:__pkg__"],
)

cc_library(
    name = "optimizer_bfgs",
    srcs = ["optimizerbfgs.cpp"],
    hdrs = ["optimizerbfgs.h"],
    deps = ["//objectivefunction:function_base",
    ":optimizer_base",],
    visibility = ["//algorithms/test:__pkg__"],
)

# cc_library(
#     name = "function_quadratic",
#     hdrs = ["costfunctionquadratic.h", "costfunctionbase.h"],
# )

# cc_library(
#     name = "function_himmelblau",
#     srcs = ["costfunctionhimmelblau.cpp"],
#     hdrs = ["costfunctionhimmelblau.h", "costfunctionbase.h"],
# )

# cc_binary(
#     name = "test_newtown",
#     srcs = ["test_newton.cpp"],
#     deps = [":optimizer_newton", 
#     ],
# )