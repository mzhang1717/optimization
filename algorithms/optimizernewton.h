#pragma once
#include <cmath>
#include <string>
#include <eigen3/Eigen/Dense>
#include <nlohmann/json.hpp>
#include "optimizerbase.h"

struct OptimizerNewtonParams {
    int max_iterations = 5000;
    int max_linesearch = 50;
    double min_step_size = 1e-14;
    double gradient_epsilon = 1e-6;
    double initial_step_size = 1.0;
    double shrink_factor = 0.9;
    double slope_factor = 1e-4;

    static OptimizerNewtonParams fromJson(const nlohmann::json& config);
    static OptimizerNewtonParams fromJsonFile(const std::string& config_path);
};

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
    OptimizerNewton(
        CostFunctionBase& costfunction,
        const Eigen::Ref<const Eigen::VectorXd>& x_ini,
        const OptimizerNewtonParams& params
    );
    ~OptimizerNewton() {}

    /// Newton direction: solve H*d = -g via LDLT; if H not PD, use -gradient_.
    void calculateSearchDirection() override;
    /// Advance iterate then recompute Hessian at new x_ (for next direction).
    void update() override;
    /// Initialize state and compute Hessian at initial point.
    void initialUpdate() override;
    /// True if A is symmetric positive definite (via Eigen::LDLT::isPositive()).
    bool isPositiveDefiniteMatrix(const Eigen::Ref<const Eigen::MatrixXd>& A);

private:
    Eigen::MatrixXd hessian_;  ///< Hessian at current x_; updated in initialUpdate() and update().
};
