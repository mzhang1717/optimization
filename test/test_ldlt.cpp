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
/*     std::vector<std::string> candidates = {
        "config.json",
        "test/config.json",
    };
    if (argc > 0 && argv[0] != nullptr) {
        const std::filesystem::path exe_path(argv[0]);
        candidates.push_back((exe_path.parent_path() / "config.json").string());
        candidates.push_back((exe_path.string() + ".runfiles/_main/test/config.json"));
    }
    if (const char* runfiles_dir = std::getenv("RUNFILES_DIR")) {
        candidates.push_back(std::string(runfiles_dir) + "/_main/test/config.json");
    }

    std::cout << "config paths: " << std::endl;
    for (const auto& path : candidates) {
        std::cout << path << std::endl;
    }

    std::ifstream config_file;
    std::string resolved_path;
    for (const auto& path : candidates) {
        config_file.open(path);
        if (config_file.is_open()) {
            resolved_path = path;
            break;
        }
        config_file.clear();
    } */

    std::ifstream config_file("test/config.json");
    if (!config_file.is_open()) {
        std::cerr << "Error: Could not open config.json" << std::endl;
        // std::cerr << "Tried paths:" << std::endl;
        // for (const auto& path : candidates) {
        //     std::cerr << "  - " << path << std::endl;
        // }
        return 1;
    }
    //std::cout << "Loaded config from: " << resolved_path << std::endl;

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



