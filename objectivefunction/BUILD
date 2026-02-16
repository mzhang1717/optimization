cc_library(
    name = "function_base",
    hdrs = ["costfunctionbase.h"],
    visibility = ["//objectivefunction/test:__pkg__",
                 "//algorithms:__pkg__",
                 "//algorithms:__subpackages__",],
)

cc_library(
    name = "function_scalar",
    srcs = ["costfunctionscalar.cpp"],
    hdrs = ["costfunctionscalar.h"],
    deps =[":function_base",],
    visibility = ["//objectivefunction/test:__pkg__",
    "//algorithms:__pkg__",
    "//algorithms:__subpackages__",],
)

cc_library(
    name = "function_quadratic",
    hdrs = ["costfunctionquadratic.h"],
    deps =[":function_base",],
    visibility = ["//objectivefunction/test:__pkg__",
    "//algorithms:__pkg__",
    "//algorithms:__subpackages__",],
)

cc_library(
    name = "function_himmelblau",
    srcs = ["costfunctionhimmelblau.cpp"],
    hdrs = ["costfunctionhimmelblau.h"],
    deps =[":function_base",],
    visibility = ["//objectivefunction/test:__pkg__",
    "//algorithms:__pkg__",
    "//algorithms:__subpackages__",],
)

cc_library(
    name = "function_mccormick",
    srcs = ["costfunctionmccormick.cpp"],
    hdrs = ["costfunctionmccormick.h"],
    deps =[":function_base",],
    visibility = ["//objectivefunction/test:__pkg__",
    "//algorithms:__pkg__",
    "//algorithms:__subpackages__",],
)

cc_library(
    name = "function_rosenbrock",
    srcs = ["costfunctionrosenbrock.cpp"],
    hdrs = ["costfunctionrosenbrock.h"],
    deps =[":function_base",],
    visibility = ["//visibility:public","//algorithms:__pkg__",
    "//algorithms:__subpackages__",],
)

cc_library(
    name = "function_convex3d",
    srcs = ["costfunctionconvex3d.cpp"],
    hdrs = ["costfunctionconvex3d.h"],
    deps = [":function_base",],
    visibility = ["//objectivefunction/test:__pkg__",
                 "//algorithms:__pkg__",
                 "//algorithms:__subpackages__",],
)
