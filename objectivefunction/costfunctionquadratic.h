#pragma once
#include "costfunctionbase.h"

/**
 * Simple 2D quadratic cost function: f(x1, x2) = 4*x1^2 + x2^2 + 1.
 *
 * Convex with minimum at (0, 0). Requires x.size() >= 2; gradient and hessian
 * must be pre-sized (2 and 2x2 respectively).
 */
class CostFunctionQuadratic : public CostFunctionBase {
public:
    CostFunctionQuadratic() {}
    ~CostFunctionQuadratic() {}

    double calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x) override {
        double x1 = x(0);
        double x2 = x(1);
        return 4 * x1 * x1 + x2 * x2 + 1;
    }
    void calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient) override {
        double x1 = x(0);
        double x2 = x(1);
        gradient(0) = 8 * x1;
        gradient(1) = 2 * x2;
    }
    void calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian) override {
        hessian(0, 0) = 8;
        hessian(0, 1) = 0;
        hessian(1, 0) = 0;
        hessian(1, 1) = 2;
    }
};