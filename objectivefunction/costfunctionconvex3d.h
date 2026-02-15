#pragma once
#include <eigen3/Eigen/Dense>
#include "costfunctionbase.h"

/**
 * Convex quadratic in 3D: f(x1, x2, x3) = x1^2 + x2^2 + x3^2.
 *
 * Hessian is 2*I (constant, positive definite). Minimum at (0, 0, 0). Requires
 * x.size() >= 3; gradient and hessian must be pre-sized (3 and 3x3).
 */
class CostFunctionConvex3D : public CostFunctionBase {
public:
    CostFunctionConvex3D() {}
    ~CostFunctionConvex3D() {}

    virtual double calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x);
    virtual void calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient);
    virtual void calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian);
};
