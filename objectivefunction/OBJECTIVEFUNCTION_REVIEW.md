# Objective Function Module – Code Review

## Summary

Review of `objectivefunction/`: base class, concrete cost functions, and test for potential bugs.

---

## Critical bugs

### 1. **CostFunctionHimmelblau::calculateHessian – wrong parameter name** ✓ Fixed

**File:** `costfunctionhimmelblau.cpp` (line 22)

The fourth parameter (the output matrix) is declared as `hessian` in the header but **named `gradient`** in the .cpp implementation. The function then does `gradient(0,0) = h11;` etc., so it is writing Hessian entries into that matrix. The caller passes the Hessian, so the **runtime behavior is correct** (the same reference is used), but the misleading name makes the code fragile and could cause wrong edits (e.g. someone “fixing” the “gradient” and breaking the Hessian).

**Fix:** Rename the fourth parameter from `gradient` to `hessian` in the .cpp to match the header and the logic.

---

### 2. **CostFunctionBase – non-virtual destructor**

**File:** `costfunctionbase.h`

The base class has a non-virtual destructor. If code does:

```cpp
CostFunctionBase* p = new CostFunctionRosenbrock();
delete p;
```

only `~CostFunctionBase()` runs; the derived destructor is never called. That is undefined behavior (e.g. if derived classes add members or resources).

**Fix:** Declare the destructor virtual: `virtual ~CostFunctionBase();` (and define it in a .cpp or `= default` in the header).

---

## Medium / robustness

### 3. **No input dimension checks**

All 2D cost functions (Quadratic, Rosenbrock, Himmelblau, McCormick) use `x(0)` and `x(1)` without checking `x.size() >= 2`. If a caller passes a vector of size 0 or 1, behavior is undefined (assertion or out-of-bounds). Similarly, gradient and hessian outputs are written with fixed indices; callers must have resized them (the current optimizers do).

**Fix:** Either document that `x.size() >= 2` and output sizes are the caller’s responsibility, or add assertions / size checks at the start of each implementation.

---

### 4. **CostFunctionExample – vector API not implemented**

**File:** `costfunctionexample.h` / `costfunctionexample.cpp`

`CostFunctionExample` only implements the **scalar** API: `calculateCostFunctionValue(double x)`, `calculateGradient(double x)`, `calculateHessian(double x)`. It does **not** override the **vector** versions that take `Eigen::Ref<const Eigen::VectorXd>& x` and output gradient/hessian. The optimizers use only the vector API. So if someone does:

```cpp
CostFunctionExample f;
OptimizerNewton opt(f, Eigen::Vector2d{1.0, 2.0});
opt.optimize();
```

the base defaults are used: value 0, zero gradient, no Hessian writes → wrong behavior.

**Fix:** Either implement the vector overloads for `CostFunctionExample` (e.g. treat `x(0)` as the scalar and ignore higher dimensions), or clearly document that this class is for scalar use only and must not be used with the optimizers.

---

## Minor / consistency

### 5. **Class name typo: McComick vs McCormick**

**File:** `costfunctionmccormick.h` / `costfunctionmccormick.cpp`

The class is named `CostFunctionMcComick` (one ‘c’). The file and BUILD target use “mccormick”. Works as-is, but the typo is confusing.

**Fix:** Rename to `CostFunctionMcCormick` (and update all references) for consistency.

---

### 6. **test_function.cpp – missing include for Rosenbrock**

**File:** `objectivefunction/test/test_function.cpp`

The test includes Quadratic, Himmelblau, McCormick, and Example, but **not** `costfunctionrosenbrock.h`. To switch the test to `CostFunctionRosenbrock`, the include would need to be added. Low impact since the test currently uses McCormick.

---

### 7. **CostFunctionQuadratic – unused include**

**File:** `costfunctionquadratic.h`

`#include <cmath>` is present but no `std::` math is used; the file only does arithmetic. Harmless but unnecessary.

---

## Summary table

| Issue                         | Severity   | Status   |
|------------------------------|------------|----------|
| Himmelblau Hessian param name| Critical   | Fixed    |
| Base destructor not virtual  | Critical   | Fixed    |
| No input dimension checks    | Medium     | Documented |
| CostFunctionExample vector API | Medium   | Documented |
| McComick typo                | Minor      | Documented |
| Test missing Rosenbrock include | Minor   | Optional fix |
| Unused &lt;cmath&gt; in Quadratic | Minor   | Optional |
