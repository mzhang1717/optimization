#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "optimizerbase.h"

/**
 * Newton's method optimizer.
 *
 * Search direction is d = -H^{-1} * g, where H is the Hessian and g the gradient.
 * The system H*d = -g is solved via LDLT (no explicit inverse). If the Hessian
 * is not positive definite at the current iterate, the direction is set to
 * -gradient_ (steepest descent fallback) and a message is printed. Requires the
 * cost function to implement calculateHessian().
 */
class OptimizerNewton : public OptimizerBase {
public:
    OptimizerNewton() {}
    OptimizerNewton(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini);
    ~OptimizerNewton() {}

    /// Newton direction: solve H*d = -g via LDLT; if H not PD, use -gradient_.
    void calculateSearchDirection();
    /// Advance iterate then recompute Hessian at new x_ (for next direction).
    void update();
    /// Initialize state and compute Hessian at initial point.
    void initialUpdate();
    /// True if A is symmetric positive definite (via Eigen::LDLT::isPositive()).
    bool isPositiveDefiniteMatrix(const Eigen::Ref<const Eigen::MatrixXd>& A);

private:
    Eigen::MatrixXd hessian_;  ///< Hessian at current x_; updated in initialUpdate() and update().
};