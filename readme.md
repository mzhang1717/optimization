# Optimization

A C++ library for **unconstrained nonlinear optimization**. It provides a common interface for objective (cost) functions and several first- and second-order optimization algorithms, with tests and demos built using [Bazel](https://bazel.build/) and [Eigen](https://eigen.tuxfamily.org/).

---

## Overview

- **Objective functions** implement value, gradient, and (optionally) Hessian via a single base interface.
- **Optimizers** take a cost function and an initial guess, then run an iterative loop (search direction, line search, update) until convergence or a maximum number of iterations.
- All numeric data use **Eigen** (`Eigen::VectorXd`, `Eigen::MatrixXd`, `Eigen::Ref<>`).

---

## Directory structure

```
optimization/
├── MODULE.bazel          # Bazel module (Bzlmod) configuration
├── algorithms/           # Optimization algorithms
│   ├── BUILD
│   ├── optimizerbase.{h,cpp}      # Abstract base; optimize() loop, line search
│   ├── optimizerdescent.{h,cpp}   # Gradient descent
│   ├── optimizernewton.{h,cpp}    # Newton's method (Hessian)
│   ├── optimizerbfgs.{h,cpp}      # BFGS quasi-Newton
│   └── test/
│       ├── BUILD
│       └── test_opt.cpp            # Demo: run optimizers on cost functions
├── objectivefunction/    # Cost / objective functions
│   ├── BUILD
│   ├── costfunctionbase.h         # Abstract interface (value, gradient, Hessian)
│   ├── costfunctionquadratic.h   # 2D quadratic
│   ├── costfunctionrosenbrock.{h,cpp}
│   ├── costfunctionhimmelblau.{h,cpp}
│   ├── costfunctionmccormick.{h,cpp}
│   ├── costfunctionconvex3d.{h,cpp}   # 3D convex (x₁² + x₂² + x₃²)
│   ├── costfunctionscalar.{h,cpp}    # Scalar example (w/ vector API)
│   └── test/
│       ├── BUILD
│       └── test_function.cpp       # Evaluate cost functions at sample points
└── test/                 # Standalone tests
    ├── BUILD
    └── test_ldlt.cpp     # testing sandbox
```

---

## Main components

### Cost functions (`objectivefunction/`)

- **Base:** `CostFunctionBase` declares (vector) API:
  - `calculateCostFunctionValue(x)` → scalar
  - `calculateGradient(x, gradient)` → writes gradient
  - `calculateHessian(x, hessian)` → writes Hessian
- **Concrete:** Quadratic, Rosenbrock, Himmelblau, McCormick, Convex3D, Scalar (single-variable). Each implements the vector overloads used by the optimizers.

### Optimizers (`algorithms/`)

- **Base:** `OptimizerBase` is abstract (pure virtual `calculateSearchDirection()`). It holds a `CostFunctionBase*`, runs `optimize()`: `initialUpdate()` → loop of `calculateSearchDirection()`, `backtrackingLineSearch()`, `update()` until `isTerminationReady()` → `showResults()`.
- **Concrete:**
  - **OptimizerDescent:** search direction = −gradient
  - **OptimizerNewton:** direction from Hessian solve (LDLT); falls back to steepest descent if Hessian is not positive definite
  - **OptimizerBFGS:** quasi-Newton with inverse-Hessian approximation and Wolfe line search

---

## Build and run

**Prerequisites:** [Bazel](https://bazel.build/) and Eigen (e.g. system install: `libeigen3-dev` on Debian/Ubuntu). The code uses `#include <eigen3/Eigen/Dense>`.

```bash
# Optimizer demo (cost functions + Newton / BFGS / descent)
bazel build //algorithms/test:test_opt
bazel run //algorithms/test:test_opt

# Cost function evaluation at sample points
bazel build //objectivefunction/test:test_function
bazel run //objectivefunction/test:test_function

# LDLT / positive-definiteness test
bazel build //test:test_ldlt
bazel run //test:test_ldlt
```

---

## Extending the codebase

**New cost function**

1. Add `costfunction<name>.{h,cpp}` in `objectivefunction/`, subclass `CostFunctionBase`, implement the vector `calculateCostFunctionValue`, `calculateGradient`, and `calculateHessian`.
2. Add a `cc_library` in `objectivefunction/BUILD` (e.g. `function_<name>`) with `deps = [":function_base"]`.
3. Use it in `test_function.cpp` and/or `test_opt.cpp`, and add the new library to the corresponding target’s `deps`.

**New optimizer**

1. Add `optimizer<name>.{h,cpp}` in `algorithms/`, subclass `OptimizerBase`, override at least `calculateSearchDirection()`.
2. Add a `cc_library` in `algorithms/BUILD` depending on `:optimizer_base` and `//objectivefunction:function_base`.
3. Instantiate and run it in `algorithms/test/test_opt.cpp`, and add the new library to the test target’s `deps`.
