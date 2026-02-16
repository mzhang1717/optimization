#include <cassert>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include "optimizerbfgs.h"

namespace {
template <typename T>
T ReadOrDefault(const nlohmann::json& section, const char* key, T default_value) {
    if (!section.contains(key)) {
        return default_value;
    }
    return section.at(key).get<T>();
}
}

OptimizerBFGSParams OptimizerBFGSParams::fromJson(const nlohmann::json& config) {
    const nlohmann::json* section = &config;
    if (config.contains("bfgs")) {
        section = &config.at("bfgs");
    } else if (config.contains("BFGS")) {
        section = &config.at("BFGS");
    }

    OptimizerBFGSParams params;
    params.max_iterations = ReadOrDefault<int>(*section, "max_iterations", params.max_iterations);
    params.max_linesearch = ReadOrDefault<int>(*section, "max_linesearch", params.max_linesearch);
    params.min_step_size = ReadOrDefault<double>(*section, "min_step_size", params.min_step_size);
    if (section->contains("gradient_epsilon")) {
        params.gradient_epsilon = section->at("gradient_epsilon").get<double>();
    } else if (section->contains("gradien_epsilon")) {
        // Backward-compatible fallback for existing config key typo.
        params.gradient_epsilon = section->at("gradien_epsilon").get<double>();
    }
    params.initial_step_size = ReadOrDefault<double>(*section, "initial_step_size", params.initial_step_size);
    params.shrink_factor = ReadOrDefault<double>(*section, "shrink_factor", params.shrink_factor);
    params.slope_factor = ReadOrDefault<double>(*section, "slope_factor", params.slope_factor);
    params.curve_factor = ReadOrDefault<double>(*section, "curve_factor", params.curve_factor);
    params.curvature_epsilon = ReadOrDefault<double>(*section, "curvature_epsilon", params.curvature_epsilon);
    return params;
}

OptimizerBFGSParams OptimizerBFGSParams::fromJsonFile(const std::string& config_path) {
    std::ifstream file(config_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open config file: " + config_path);
    }
    nlohmann::json config;
    file >> config;
    return fromJson(config);
}

OptimizerBFGS::OptimizerBFGS(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini)
    : OptimizerBFGS(costfunction, x_ini, OptimizerBFGSParams{}) {}

OptimizerBFGS::OptimizerBFGS(
    CostFunctionBase& costfunction,
    const Eigen::Ref<const Eigen::VectorXd>& x_ini,
    const OptimizerBFGSParams& params
) : OptimizerBase{costfunction, x_ini} {
    hessian_inverse_.resize(x_ini.rows(), x_ini.rows());
    max_iterations_ = params.max_iterations;
    max_linesearch_ = params.max_linesearch;
    min_step_size_ = params.min_step_size;
    gradient_epsilon_ = params.gradient_epsilon;
    initial_step_size_ = params.initial_step_size;
    shrink_factor_ = params.shrink_factor;
    slope_factor_ = params.slope_factor;
    curve_factor_ = params.curve_factor;
    curvature_epsilon_ = params.curvature_epsilon;
    assert(curve_factor_ > slope_factor_);  // Required for Wolfe curvature condition.
}

void OptimizerBFGS::calculateSearchDirection() {
    search_direction_ = -hessian_inverse_ * gradient_;
}

void OptimizerBFGS::initialUpdate() {
    OptimizerBase::initialUpdate();
    int n = gradient_.rows();
    hessian_inverse_ = Eigen::MatrixXd::Identity(n, n);
}

void OptimizerBFGS::update() {
    Eigen::VectorXd x_previous_ = x_;
    Eigen::VectorXd gradient_previous = gradient_;

    OptimizerBase::update();

    // BFGS vectors: s = x_new - x_old, y = g_new - g_old.
    Eigen::VectorXd s = x_ - x_previous_;
    Eigen::VectorXd y = gradient_ - gradient_previous;

    // Skip update if y'*s not sufficiently positive (avoids division by zero / indefinite H).
    const double y_dot_s = y.dot(s);
    if (y_dot_s <= curvature_epsilon_) {
        return;
    }
    double rho = 1 / y_dot_s;

    // BFGS formula: H_new = (I - rho*s*y')*H*(I - rho*y*s') + rho*s*s'.
    Eigen::MatrixXd H = hessian_inverse_;
    int n = H.rows();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    hessian_inverse_ = (I - rho * s * y.transpose()) * H * (I - rho * y * s.transpose())
                       + rho * s * s.transpose();
}

void OptimizerBFGS::backtrackingLineSearch() {
    step_size_ = initial_step_size_;
    double directional_derivative = gradient_.transpose() * search_direction_;

    Eigen::VectorXd gradient_next = gradient_;
    ptr_cost_function_->calculateGradient(x_ + step_size_ * search_direction_, gradient_next);
    double function_value_next = ptr_cost_function_->calculateCostFunctionValue(x_ + step_size_ * search_direction_);

    int num_linesearch = 0;

    // Wolfe conditions: Armijo (sufficient decrease) and curvature (gradient_next'*p >= curve_factor_ * directional_derivative).
    while (((function_value_next > (function_value_ + slope_factor_ * step_size_ * directional_derivative))
            || (gradient_next.transpose() * search_direction_ < curve_factor_ * directional_derivative))
           && num_linesearch <= max_linesearch_ && step_size_ >= min_step_size_) {
        step_size_ *= shrink_factor_;
        ptr_cost_function_->calculateGradient(x_ + step_size_ * search_direction_, gradient_next);
        function_value_next = ptr_cost_function_->calculateCostFunctionValue(x_ + step_size_ * search_direction_);
        num_linesearch++;
    }

    if (step_size_ < min_step_size_) {
        std::cout << "At " << number_iterations_ << "th iteration, step_size_ = " << step_size_ << std::endl;
    }
}
