# cc_library(
#     name = "optimizer_newton",
#     srcs = ["optimizernewton.cpp","optimizerbase.cpp"],
#     hdrs = ["optimizernewton.h", "optimizerbase.h"],
# )

# cc_library(
#     name = "function_quadratic",
#     hdrs = ["costfunctionquadratic.h", "costfunctionbase.h"],
# )

# cc_library(
#     name = "function_himmelblau",
#     srcs = ["costfunctionhimmelblau.cpp"],
#     hdrs = ["costfunctionhimmelblau.h", "costfunctionbase.h"],
# )

cc_binary(
    name = "test_opt",
    srcs = ["test_opt.cpp"],
    data = ["optimizer_config.json"],
    deps = ["//algorithms:optimizer_base",
            "//algorithms:optimizer_descent",
            "//algorithms:optimizer_newton",
            "//algorithms:optimizer_bfgs",
            "//objectivefunction:function_convex3d",
            "//objectivefunction:function_himmelblau",
            "//objectivefunction:function_mccormick",
            "//objectivefunction:function_quadratic",
            "//objectivefunction:function_rosenbrock",
            "//objectivefunction:function_scalar",
    ],
)
