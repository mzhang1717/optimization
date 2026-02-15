#pragma once

#include <eigen3/Eigen/Dense>
#include "../objectivefunction/costfunctionbase.h"
#include "optimizerbase.h"

/**
 * Steepest descent (gradient descent) optimizer.
 *
 * Search direction is the negative gradient: d = -gradient_, so each step
 * moves in the direction of greatest local decrease. Uses the base class
 * Armijo backtracking line search. Only requires the cost function to
 * provide value and gradient (no Hessian).
 */
class OptimizerDescent : public OptimizerBase {
public:
    OptimizerDescent() {}
    OptimizerDescent(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini);
    ~OptimizerDescent() {}

    /// Set search_direction_ = -gradient_ (steepest descent direction).
    void calculateSearchDirection();
};