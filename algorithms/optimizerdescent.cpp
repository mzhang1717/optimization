#include <fstream>
#include <stdexcept>
#include "optimizerdescent.h"

namespace {
template <typename T>
T ReadOrDefault(const nlohmann::json& section, const char* key, T default_value) {
    if (!section.contains(key)) {
        return default_value;
    }
    return section.at(key).get<T>();
}
}

OptimizerDescentParams OptimizerDescentParams::fromJson(const nlohmann::json& config) {
    const nlohmann::json& section = config.contains("gradient_descent") ? config.at("gradient_descent") : config;

    OptimizerDescentParams params;
    params.max_iterations = ReadOrDefault<int>(section, "max_iterations", params.max_iterations);
    params.max_linesearch = ReadOrDefault<int>(section, "max_linesearch", params.max_linesearch);
    params.min_step_size = ReadOrDefault<double>(section, "min_step_size", params.min_step_size);
    params.gradient_epsilon = ReadOrDefault<double>(section, "gradient_epsilon", params.gradient_epsilon);
    params.initial_step_size = ReadOrDefault<double>(section, "initial_step_size", params.initial_step_size);
    params.shrink_factor = ReadOrDefault<double>(section, "shrink_factor", params.shrink_factor);
    params.slope_factor = ReadOrDefault<double>(section, "slope_factor", params.slope_factor);
    return params;
}

OptimizerDescentParams OptimizerDescentParams::fromJsonFile(const std::string& config_path) {
    std::ifstream file(config_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open config file: " + config_path);
    }
    nlohmann::json config;
    file >> config;
    return fromJson(config);
}

OptimizerDescent::OptimizerDescent(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini)
    : OptimizerDescent(costfunction, x_ini, OptimizerDescentParams{}) {}

OptimizerDescent::OptimizerDescent(
    CostFunctionBase& costfunction,
    const Eigen::Ref<const Eigen::VectorXd>& x_ini,
    const OptimizerDescentParams& params
) : OptimizerBase{costfunction, x_ini} {
    max_iterations_ = params.max_iterations;
    max_linesearch_ = params.max_linesearch;
    min_step_size_ = params.min_step_size;
    gradient_epsilon_ = params.gradient_epsilon;
    initial_step_size_ = params.initial_step_size;
    shrink_factor_ = params.shrink_factor;
    slope_factor_ = params.slope_factor;
}

void OptimizerDescent::calculateSearchDirection() {
    search_direction_ = -gradient_;
}
