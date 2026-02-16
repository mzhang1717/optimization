#pragma once

#include <string>
#include <eigen3/Eigen/Dense>
#include <nlohmann/json.hpp>
#include "../objectivefunction/costfunctionbase.h"
#include "optimizerbase.h"

struct OptimizerDescentParams {
    int max_iterations = 5000;
    int max_linesearch = 50;
    double min_step_size = 1e-14;
    double gradient_epsilon = 1e-6;
    double initial_step_size = 1.0;
    double shrink_factor = 0.5;
    double slope_factor = 0.5;

    static OptimizerDescentParams fromJson(const nlohmann::json& config);
    static OptimizerDescentParams fromJsonFile(const std::string& config_path);
};

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
    OptimizerDescent(
        CostFunctionBase& costfunction,
        const Eigen::Ref<const Eigen::VectorXd>& x_ini,
        const OptimizerDescentParams& params
    );
    ~OptimizerDescent() {}

    /// Set search_direction_ = -gradient_ (steepest descent direction).
    void calculateSearchDirection() override;
};
