#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "costfunctionbase.h"

/**
 * Himmelblau's function (2D): f(x1, x2) = (x1^2 + x2 - 11)^2 + (x1 + x2^2 - 7)^2.
 *
 * Has four local minima. Requires x.size() >= 2; gradient and hessian must be
 * pre-sized (2 and 2x2 respectively).
 */
class CostFunctionHimmelblau : public CostFunctionBase {
public:
    CostFunctionHimmelblau() {}
    ~CostFunctionHimmelblau() {}

    virtual double calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x);
    virtual void calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient);
    virtual void calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian);
};