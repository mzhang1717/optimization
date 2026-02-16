#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <cstdlib>
#include <nlohmann/json.hpp>
///#include <eigen3/Eigen/Dense>

using json = nlohmann::json;

int main(int argc, char** argv) {
    /// Read config file from common local and Bazel runfiles locations.
    std::ifstream config_file("test/config.json");
    if (!config_file.is_open()) {
        std::cerr << "Error: Could not open config.json" << std::endl;
        return 1;
    }

    /// Parse config file
    json data;
    try {
        config_file >> data;
    } catch (json::parse_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    /// Extract parameters from config file
    int max_iterations = data["gradient_descent"]["max_iterations"];
    int max_linesearch = data["gradient_descent"]["max_linesearch"];
    double min_step_size = data["gradient_descent"]["min_step_size"];
    double gradient_epsilon = data["gradient_descent"]["gradient_epsilon"];
    double initial_step_size = data["gradient_descent"]["initial_step_size"];
    double shrink_factor = data["gradient_descent"]["shrink_factor"];
    double slope_factor = data["gradient_descent"]["slope_factor"];

    /// Print parameters
    std::cout << "Max iterations: " << max_iterations << std::endl;
    std::cout << "Max linesearch: " << max_linesearch << std::endl;
    std::cout << "Min step size: " << min_step_size << std::endl;
    std::cout << "Gradient epsilon: " << gradient_epsilon << std::endl;
    std::cout << "Initial step size: " << initial_step_size << std::endl;
    std::cout << "Shrink factor: " << shrink_factor << std::endl;
    std::cout << "Slope factor: " << slope_factor << std::endl;

    return 0;
}



