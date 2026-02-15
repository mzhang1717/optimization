#include <iostream>
#include <memory>
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
//#include "costfunctionquadratic.h"

int main () {

    // ----- CostFunctionConvex3D (3D: f(x) = x1^2 + x2^2 + x3^2, minimum at (0,0,0)) -----
    std::cout << "======= CostFunctionConvex3D =======" << std::endl;
    CostFunctionConvex3D convex3d_function;
    std::unique_ptr<OptimizerBase> ptrOptimizer;

    std::cout << "------- Newton method -----" << std::endl;
    ptrOptimizer.reset(new OptimizerNewton(convex3d_function, Eigen::Vector3d{1.0, 1.0, 1.0}));
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::Vector3d{-2.0, 3.0, 1.0});
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::Vector3d{5.0, -4.0, 2.0});
    ptrOptimizer->optimize();

    std::cout << "------- BFGS method -----" << std::endl;
    ptrOptimizer.reset(new OptimizerBFGS(convex3d_function, Eigen::Vector3d{1.0, 1.0, 1.0}));
    ptrOptimizer->optimize();
    ptrOptimizer->setInitialGuess(Eigen::Vector3d{-2.0, 3.0, 1.0});
    ptrOptimizer->optimize();

    std::cout << "------- Gradient Descent -----" << std::endl;
    ptrOptimizer.reset(new OptimizerDescent(convex3d_function, Eigen::Vector3d{1.0, 1.0, 1.0}));
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
    ptrOptimizer.reset(new OptimizerNewton(myfunction, Eigen::Vector2d{2.1, 1.0}));

    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-4,5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-1,-5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{6, -3});
    ptrOptimizer->optimize();

    std::cout << "------- BFGS method -----" << std::endl;
    ptrOptimizer.reset(new OptimizerBFGS(myfunction, Eigen::Vector2d{-1.2, 1.0}));

    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-4,5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-1,-5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{6,-3});
    ptrOptimizer->optimize();

    std::cout << "------- Gradient Descent Method -----" << std::endl;
    ptrOptimizer.reset(new OptimizerDescent(myfunction, Eigen::Vector2d{2.0, 1.0}));

    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-4,5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-1,-5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{6,-3});
    ptrOptimizer->optimize();

    return 0;
}