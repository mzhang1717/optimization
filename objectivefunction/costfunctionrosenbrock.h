#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "costfunctionbase.h"

/**
 * Rosenbrock function (2D): f(x1, x2) = (1 - x1)^2 + 100*(x2 - x1^2)^2.
 *
 * Classic test function with a narrow valley; minimum at (1, 1). Requires
 * x.size() >= 2; gradient and hessian must be pre-sized (2 and 2x2).
 */
class CostFunctionRosenbrock : public CostFunctionBase {
public:
    CostFunctionRosenbrock() {}
    ~CostFunctionRosenbrock() {}

    virtual double calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x);
    virtual void calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient);
    virtual void calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian);
};