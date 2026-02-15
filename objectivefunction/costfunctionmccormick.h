#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "costfunctionbase.h"

/**
 * McCormick function (2D): f(x1, x2) = sin(x1 + x2) + (x1 - x2)^2 - 1.5*x1 + 2.5*x2 + 1.
 *
 * Test function with a global minimum in the box -1.5 <= x1 <= 4, -3 <= x2 <= 4.
 * Requires x.size() >= 2; gradient and hessian must be pre-sized (2 and 2x2).
 */
class CostFunctionMcCormick : public CostFunctionBase {
public:
    CostFunctionMcCormick() {}
    ~CostFunctionMcCormick() {}

    virtual double calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x);
    virtual void calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient);
    virtual void calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian);
};