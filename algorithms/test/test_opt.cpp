#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <filesystem>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include "../optimizerbase.h"
#include "../optimizerdescent.h"
#include "../optimizernewton.h"
#include "../optimizerbfgs.h"
#include "../../objectivefunction/costfunctionhimmelblau.h"
#include "../../objectivefunction/costfunctionmccormick.h"
#include "../../objectivefunction/costfunctionquadratic.h"
#include "../../objectivefunction/costfunctionrosenbrock.h"
#include "../../objectivefunction/costfunctionconvex3d.h"
#include "../../objectivefunction/costfunctionscalar.h"
//#include "costfunctionquadratic.h"

namespace {
std::string ResolveConfigPath(int argc, char** argv) {
    std::vector<std::string> candidates = {
        "optimizer_config.json",
        "algorithms/test/optimizer_config.json",
    };
    if (argc > 0 && argv[0] != nullptr) {
        const std::filesystem::path exe_path(argv[0]);
        candidates.push_back((exe_path.parent_path() / "optimizer_config.json").string());
        candidates.push_back((exe_path.string() + ".runfiles/_main/algorithms/test/optimizer_config.json"));
    }
    if (const char* runfiles_dir = std::getenv("RUNFILES_DIR")) {
        candidates.push_back(std::string(runfiles_dir) + "/_main/algorithms/test/optimizer_config.json");
    }

    for (const auto& path : candidates) {
        std::ifstream file(path);
        if (file.is_open()) {
            return path;
        }
    }

    std::cerr << "Error: Could not open optimizer_config.json" << std::endl;
    std::cerr << "Tried paths:" << std::endl;
    for (const auto& path : candidates) {
        std::cerr << "  - " << path << std::endl;
    }
    return {};
}
}

int main(int argc, char** argv) {
    const std::string config_path = ResolveConfigPath(argc, argv);
    if (config_path.empty()) {
        return 1;
    }
    std::cout << "Loaded optimizer config from: " << config_path << std::endl;

    const OptimizerDescentParams descent_params = OptimizerDescentParams::fromJsonFile(config_path);
    const OptimizerNewtonParams newton_params = OptimizerNewtonParams::fromJsonFile(config_path);
    const OptimizerBFGSParams bfgs_params = OptimizerBFGSParams::fromJsonFile(config_path);

    // ----- CostFunctionConvex3D (3D: f(x) = x1^2 + x2^2 + x3^2, minimum at (0,0,0)) -----
    std::cout << "======= CostFunctionConvex3D =======" << std::endl;
    CostFunctionConvex3D convex3d_function;
    std::unique_ptr<OptimizerBase> ptrOptimizer;

    std::cout << "------- Newton method -----" << std::endl;
    ptrOptimizer.reset(new OptimizerNewton(convex3d_function, Eigen::Vector3d{1.0, 1.0, 1.0}, newton_params));
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::Vector3d{-2.0, 3.0, 1.0});
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::Vector3d{5.0, -4.0, 2.0});
    ptrOptimizer->optimize();

    std::cout << "------- BFGS method -----" << std::endl;
    ptrOptimizer.reset(new OptimizerBFGS(convex3d_function, Eigen::Vector3d{1.0, 1.0, 1.0}, bfgs_params));
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::Vector3d{-2.0, 3.0, 1.0});
    ptrOptimizer->optimize();

    std::cout << "------- Gradient Descent -----" << std::endl;
    ptrOptimizer.reset(new OptimizerDescent(convex3d_function, Eigen::Vector3d{1.0, 1.0, 1.0}, descent_params));
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::Vector3d{-2.0, 3.0, 1.0});
    ptrOptimizer->optimize();

    // ----- 2D cost function (e.g. McCormick) -----
    std::cout << "======= CostFunctionMcCormick (2D) =======" << std::endl;
    //CostFunctionQuadratic myfunction;
    //CostFunctionHimmelblau myfunction;
    CostFunctionMcCormick myfunction;
    //CostFunctionRosenbrock myfunction;

    std::cout << "------- Newton method -----" << std::endl;
    ptrOptimizer.reset(new OptimizerNewton(myfunction, Eigen::Vector2d{2.1, 1.0}, newton_params));

    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-4,5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-1,-5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{6, -3});
    ptrOptimizer->optimize();

    std::cout << "------- BFGS method -----" << std::endl;
    ptrOptimizer.reset(new OptimizerBFGS(myfunction, Eigen::Vector2d{-1.2, 1.0}, bfgs_params));

    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-4,5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-1,-5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{6,-3});
    ptrOptimizer->optimize();

    std::cout << "------- Gradient Descent Method -----" << std::endl;
    ptrOptimizer.reset(new OptimizerDescent(myfunction, Eigen::Vector2d{2.0, 1.0}, descent_params));

    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-4,5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-1,-5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{6,-3});
    ptrOptimizer->optimize();

    // ----- 1D scalar cost function -----
    std::cout << "======= CostFunctionScalar (1D) =======" << std::endl;
    CostFunctionScalar scalar_function;

    std::cout << "------- Newton method -----" << std::endl;
    ptrOptimizer.reset(new OptimizerNewton(scalar_function, Eigen::VectorXd::Constant(1, 2.0), newton_params));
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::VectorXd::Constant(1, 4.0));
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::VectorXd::Constant(1, 6.0));
    ptrOptimizer->optimize();

    std::cout << "------- BFGS method -----" << std::endl;
    ptrOptimizer.reset(new OptimizerBFGS(scalar_function, Eigen::VectorXd::Constant(1, 2.0), bfgs_params));
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::VectorXd::Constant(1, 4.0));
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::VectorXd::Constant(1, 6.0));
    ptrOptimizer->optimize();

    std::cout << "------- Gradient Descent method -----" << std::endl;
    ptrOptimizer.reset(new OptimizerDescent(scalar_function, Eigen::VectorXd::Constant(1, 2.0), descent_params));
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::VectorXd::Constant(1, 4.0));
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::VectorXd::Constant(1, 6.0));
    ptrOptimizer->optimize();

    return 0;
}
