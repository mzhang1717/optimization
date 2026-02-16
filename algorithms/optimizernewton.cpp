#include <iostream>
#include <fstream>
#include <stdexcept>
#include "optimizernewton.h"

namespace {
template <typename T>
T ReadOrDefault(const nlohmann::json& section, const char* key, T default_value) {
    if (!section.contains(key)) {
        return default_value;
    }
    return section.at(key).get<T>();
}
}

OptimizerNewtonParams OptimizerNewtonParams::fromJson(const nlohmann::json& config) {
    const nlohmann::json& section = config.contains("newton") ? config.at("newton") : config;

    OptimizerNewtonParams params;
    params.max_iterations = ReadOrDefault<int>(section, "max_iterations", params.max_iterations);
    params.max_linesearch = ReadOrDefault<int>(section, "max_linesearch", params.max_linesearch);
    params.min_step_size = ReadOrDefault<double>(section, "min_step_size", params.min_step_size);
    params.gradient_epsilon = ReadOrDefault<double>(section, "gradient_epsilon", params.gradient_epsilon);
    params.initial_step_size = ReadOrDefault<double>(section, "initial_step_size", params.initial_step_size);
    params.shrink_factor = ReadOrDefault<double>(section, "shrink_factor", params.shrink_factor);
    params.slope_factor = ReadOrDefault<double>(section, "slope_factor", params.slope_factor);
    return params;
}

OptimizerNewtonParams OptimizerNewtonParams::fromJsonFile(const std::string& config_path) {
    std::ifstream file(config_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open config file: " + config_path);
    }
    nlohmann::json config;
    file >> config;
    return fromJson(config);
}

OptimizerNewton::OptimizerNewton(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini)
    : OptimizerNewton(costfunction, x_ini, OptimizerNewtonParams{}) {}

OptimizerNewton::OptimizerNewton(
    CostFunctionBase& costfunction,
    const Eigen::Ref<const Eigen::VectorXd>& x_ini,
    const OptimizerNewtonParams& params
) : OptimizerBase{costfunction, x_ini} {
    hessian_.resize(x_ini.rows(), x_ini.rows());
    max_iterations_ = params.max_iterations;
    max_linesearch_ = params.max_linesearch;
    min_step_size_ = params.min_step_size;
    gradient_epsilon_ = params.gradient_epsilon;
    initial_step_size_ = params.initial_step_size;
    shrink_factor_ = params.shrink_factor;
    slope_factor_ = params.slope_factor;
}

void OptimizerNewton::calculateSearchDirection() {
    if (isPositiveDefiniteMatrix(hessian_)) {
        // Solve H * d = -g via LDLT (numerically more stable than forming H^{-1}).
        search_direction_ = -hessian_.ldlt().solve(gradient_);
    } else {
        std::cout << "At " << number_iterations_ << "th iteration: Hessian is NOT positive definite!" << std::endl;
        std::cout << "Hessian = " << std::endl << hessian_ << std::endl;
        search_direction_ = -gradient_;  // Fallback to steepest descent.
    }
}

void OptimizerNewton::update() {
    OptimizerBase::update();
    ptr_cost_function_->calculateHessian(x_, hessian_);
}

void OptimizerNewton::initialUpdate() {
    OptimizerBase::initialUpdate();
    ptr_cost_function_->calculateHessian(x_, hessian_);
}

bool OptimizerNewton::isPositiveDefiniteMatrix(const Eigen::Ref<const Eigen::MatrixXd>& A) {
    Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
    return ldlt.isPositive();
}
