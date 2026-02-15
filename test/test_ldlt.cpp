#include <iostream>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>
///#include <eigen3/Eigen/Dense>

using json = nlohmann::json;

int main() {
    /// Read config file
    std::ifstream config_file("config.json");
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
    std::string optimizer_type = data["algorithms"]["gradient_descent"]["type"];
    double step_size = data["algorithms"]["gradient_descent"]["parameters"]["step_size"];
    int max_iterations = data["algorithms"]["gradient_descent"]["parameters"]["max_iterations"];
    int max_linesearch = data["algorithms"]["gradient_descent"]["parameters"]["max_linesearch"];

    /// Print parameters
    std::cout << "Optimizer type: " << optimizer_type << std::endl;
    std::cout << "Step size: " << step_size << std::endl;
    std::cout << "Max iterations: " << max_iterations << std::endl;
    std::cout << "Max linesearch: " << max_linesearch << std::endl;

    return 0;
}




