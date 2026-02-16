#pragma once
#include <cmath>
#include <string>
#include <eigen3/Eigen/Dense>
#include <nlohmann/json.hpp>
#include "optimizerbase.h"

struct OptimizerBFGSParams {
    int max_iterations = 5000;
    int max_linesearch = 50;
    double min_step_size = 1e-14;
    double gradient_epsilon = 1e-6;
    double initial_step_size = 1.0;
    double shrink_factor = 0.9;
    double slope_factor = 1e-4;
    double curve_factor = 0.9;
    double curvature_epsilon = 1e-10;

    static OptimizerBFGSParams fromJson(const nlohmann::json& config);
    static OptimizerBFGSParams fromJsonFile(const std::string& config_path);
};

/**
 * BFGS quasi-Newton optimizer.
 *
 * Maintains an approximation H of the inverse Hessian and uses search direction
 * d = -H * gradient_. H is updated each iteration via the BFGS formula so that
 * it satisfies the secant condition. Initial H is the identity. Uses a
 * Wolfe-condition line search (Armijo + curvature) instead of the base class
 * Armijo-only search. If y'*s is not sufficiently positive, the inverse Hessian
 * update is skipped to avoid numerical issues. Only requires the cost function
 * to provide value and gradient (no Hessian).
 */
class OptimizerBFGS : public OptimizerBase {
public:
    OptimizerBFGS() {}
    OptimizerBFGS(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini);
    OptimizerBFGS(
        CostFunctionBase& costfunction,
        const Eigen::Ref<const Eigen::VectorXd>& x_ini,
        const OptimizerBFGSParams& params
    );
    ~OptimizerBFGS() {}

    /// Set search_direction_ = -hessian_inverse_ * gradient_.
    void calculateSearchDirection() override;
    /// Advance iterate, then update hessian_inverse_ via BFGS formula (with y'*s safeguard).
    void update() override;
    /// Initialize state and set hessian_inverse_ = I.
    void initialUpdate() override;
    /// Wolfe line search: Armijo + curvature condition; shrinks step until both hold or limits hit.
    void backtrackingLineSearch() override;

private:
    Eigen::MatrixXd hessian_inverse_;  ///< Approximation to inverse Hessian; updated in update().
    double curvature_epsilon_;         ///< Safeguard threshold for y'*s in BFGS update.
};
