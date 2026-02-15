#pragma once
#include <eigen3/Eigen/Dense>

/**
 * Abstract base class for scalar cost/objective functions.
 *
 * Provides both a scalar API (single double x) and a vector API (Eigen::VectorXd).
 * The optimizers use only the vector API: calculateCostFunctionValue(x),
 * calculateGradient(x, gradient), and calculateHessian(x, hessian). Callers must
 * ensure x, gradient, and hessian have the correct sizes for the concrete function.
 */
class CostFunctionBase {
public:
    CostFunctionBase() {}
    virtual ~CostFunctionBase() = default;

    // --- Scalar API (optional; default implementations return 0) ---
    virtual double calculateCostFunctionValue(double x) { return 0; }
    virtual double calculateGradient(double x) { return 0; }
    virtual double calculateHessian(double x) { return 0; }

    // --- Vector API (used by optimizers; override in concrete classes) ---
    virtual double calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x) { return 0; }
    virtual void calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient) { return; }
    virtual void calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian) { return; }
};