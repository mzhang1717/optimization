#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "optimizerbase.h"

/**
 * BFGS quasi-Newton optimizer.
 *
 * Maintains an approximation H of the inverse Hessian and uses search direction
 * d = -H * gradient_. H is updated each iteration via the BFGS formula so that
 * it satisfies the secant condition. Initial H is the identity. Uses a
 * Wolfe-condition line search (Armijo + curvature) instead of the base class
 * Armijo-only search. If y'*s is not sufficiently positive, the inverse Hessian
 * update is skipped to avoid numerical issues. Only requires the cost function
 * to provide value and gradient (no Hessian).
 */
class OptimizerBFGS : public OptimizerBase {
public:
    OptimizerBFGS() {}
    OptimizerBFGS(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini);
    ~OptimizerBFGS() {}

    /// Set search_direction_ = -hessian_inverse_ * gradient_.
    void calculateSearchDirection() override;
    /// Advance iterate, then update hessian_inverse_ via BFGS formula (with y'*s safeguard).
    void update() override;
    /// Initialize state and set hessian_inverse_ = I.
    void initialUpdate() override;
    /// Wolfe line search: Armijo + curvature condition; shrinks step until both hold or limits hit.
    void backtrackingLineSearch() override;

private:
    Eigen::MatrixXd hessian_inverse_;  ///< Approximation to inverse Hessian; updated in update().
};