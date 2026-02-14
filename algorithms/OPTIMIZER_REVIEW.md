# OptimizerBase and Child Classes – Code Review

## Summary

Review of `OptimizerBase`, `OptimizerDescent`, `OptimizerNewton`, and `OptimizerBFGS` for potential bugs and robustness issues.

---

## Critical bugs

### 1. **Base backtracking line search can loop forever**  
**File:** `optimizerbase.cpp` – `backtrackingLineSearch()`

The `while` loop has no iteration limit or minimum step size. If the direction is not a descent direction, or the cost never satisfies the Armijo condition (e.g. numerical issues), the loop never exits.

**Fix:** Cap the number of shrinkage steps (e.g. use `max_linesearch_`) and/or enforce a minimum step size, then break and use the current step (or fail explicitly).

---

### 2. **BFGS: division by zero / unstable update when `y.dot(s)` is tiny**  
**File:** `optimizerbfgs.cpp` – `update()`

```cpp
double rho = 1/(y.dot(s));
```

- If `y.dot(s) == 0`, this is undefined.
- If `y.dot(s) < 0`, the inverse Hessian approximation can become indefinite.
- If `y.dot(s)` is very small but positive, `rho` is huge and the update can blow up.

**Fix:** Only apply the BFGS update when `y.dot(s) > epsilon` (e.g. `1e-10`). Otherwise skip the update (keep current `hessian_inverse_`) or fall back to a safe step.

---

### 3. **Newton: explicit inverse and numerical stability**  
**File:** `optimizernewton.cpp` – `calculateSearchDirection()`

```cpp
search_direction_ = -hessian_.inverse() * gradient_;
```

- Forming `hessian_.inverse()` is O(n³) and less numerically stable than solving a linear system.
- Prefer solving `hessian_ * d = -gradient_` (e.g. LDLT or Cholesky) instead of multiplying by the inverse.

**Fix:** Use e.g. `search_direction_ = -hessian_.ldlt().solve(gradient_);` (or Cholesky if you know the matrix is PD).

---

## Medium / robustness issues

### 4. **Uninitialized members in `OptimizerBase`** ✓ Addressed

- **Default constructor:** `OptimizerBase()` leaves many members uninitialized: `function_value_`, `step_size_`, `gradient_epsilon_`, etc.
- **Two-argument constructor:** Does not set line-search or tolerance parameters; only subclasses set them.

**Fix applied:** `OptimizerBase` is now **abstract**: `calculateSearchDirection()` is declared pure virtual (`= 0`). That prevents direct instantiation, so the uninitialized-parameters code path cannot be used; only derived classes (which set all parameters in their constructors) are instantiable.

---

### 5. **`search_direction_` never resized in base**

`search_direction_` is only assigned in subclasses’ `calculateSearchDirection()`. The base never resizes it. Current call order in `optimize()` (e.g. `calculateSearchDirection()` then `backtrackingLineSearch()`) avoids using an uninitialized vector, but any reorder or new code path could use a default-constructed (size 0) `search_direction_`.

**Fix:** In `initialUpdate()`, resize `search_direction_` to match `gradient_.size()` (e.g. `search_direction_.resizeLike(gradient_);`) so it is always valid before the first line search.

---

### 6. **BFGS line search: possible underflow of `step_size_`**

BFGS’s `backtrackingLineSearch()` respects `max_linesearch_` but does not enforce a minimum step size. After many shrinkages, `step_size_` can underflow to zero or denormals, leading to no progress or NaN.

**Fix:** Inside the loop, enforce a minimum step size (e.g. `1e-14`) and break if `step_size_` would go below it, then use that minimum (or treat as line-search failure).

---

## Minor / clarity

### 7. **Comment typo in base backtracking**

Comment says “f(x - step size * p)” but the code uses `x_ + step_size_ * search_direction_`. For descent, `p = -gradient`, so it’s equivalent to “x - step*gradient”, but the comment is inconsistent with the code. Prefer “f(x + step_size_ * search_direction_)” in the comment.

---

### 8. **Variable name `gradient_square_negative`**

The variable holds `gradient_.transpose() * search_direction_` (directional derivative), not “gradient squared” or “negative gradient squared”. For steepest descent it happens to be `-||gradient||²`. Renaming to e.g. `directional_derivative` would better match the Armijo condition and avoid confusion for non-descent directions.

---

### 9. **Newton: positive definiteness via leading principal minors**

`isPositiveDefiniteMatrix()` checks determinants of leading principal submatrices, which is correct in exact arithmetic but can be unstable for large or ill-conditioned matrices. For production use, consider LDLT (success implies symmetric positive definite) or checking eigenvalues.

---

## Suggested fix order

1. Add a maximum iteration (and optionally minimum step) guard in base `backtrackingLineSearch()`.
2. In BFGS `update()`, guard `y.dot(s)` and only update when `y.dot(s) > epsilon`.
3. In Newton `calculateSearchDirection()`, replace `hessian_.inverse() * gradient_` with an LDLT (or Cholesky) solve.
4. Optionally: initialize all `OptimizerBase` parameters in constructors and resize `search_direction_` in `initialUpdate()` for robustness.
