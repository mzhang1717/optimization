#pragma once

#include "costfunctionbase.h"

/**
 * Example scalar cost function: polynomial f(x) = x^8 - 27*x^7 + ... (single variable).
 *
 * Implements both the scalar API (double x) and the vector API by treating x(0)
 * as the scalar. Compatible with optimizers when used with a 1D initial guess
 * (e.g. Eigen::Vector1d or Eigen::VectorXd of size 1).
 */
class CostFunctionExample : public CostFunctionBase {
public:
    CostFunctionExample() {}
    ~CostFunctionExample() {}

    double calculateCostFunctionValue(double x);
    double calculateGradient(double x);
    double calculateHessian(double x);

    // Vector API (1D): delegates to scalar implementation using x(0).
    double calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x) override;
    void calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient) override;
    void calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian) override;
};