#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "../costfunctionexample.h"
#include "../costfunctionquadratic.h"
#include "../costfunctionhimmelblau.h"
#include "../costfunctionmccormick.h"
#include "../costfunctionrosenbrock.h"
#include "../costfunctionconvex3d.h"

// Helper: evaluate and print value, gradient, Hessian for a 2D cost function at given points.
void test_2d_function(const char* name, CostFunctionBase* pFunc,
                     const std::vector<Eigen::Vector2d>& points) {
    std::cout << "=== " << name << " ===" << std::endl;
    Eigen::Vector2d gradient;
    Eigen::Matrix2d hessian;
    for (const auto& x : points) {
        double f = pFunc->calculateCostFunctionValue(x);
        pFunc->calculateGradient(x, gradient);
        pFunc->calculateHessian(x, hessian);
        std::cout << "x = " << std::endl << x << std::endl
                  << "f(x) = " << f << std::endl
                  << "g(x) = " << std::endl << gradient << std::endl
                  << "H(x) = " << std::endl << hessian << std::endl << std::endl;
    }
}

int main() {

    // ----- CostFunctionConvex3D (3D: f = x1^2 + x2^2 + x3^2) -----
    std::cout << "=== CostFunctionConvex3D ===" << std::endl;
    CostFunctionConvex3D convex3d_function;
    std::vector<Eigen::Vector3d> x3_vec = {
        {0.0, 0.0, 0.0},
        {1.0, 2.0, 3.0},
        {-1.0, 0.5, -2.0}
    };
    Eigen::Vector3d gradient3;
    Eigen::Matrix3d hessian3;
    for (const auto& x : x3_vec) {
        double f = convex3d_function.calculateCostFunctionValue(x);
        convex3d_function.calculateGradient(x, gradient3);
        convex3d_function.calculateHessian(x, hessian3);
        std::cout << "x = " << std::endl << x << std::endl
                  << "f(x) = " << f << std::endl
                  << "g(x) = " << std::endl << gradient3 << std::endl
                  << "H(x) = " << std::endl << hessian3 << std::endl << std::endl;
    }

    // ----- 2D functions: shared test points -----
    std::vector<Eigen::Vector2d> x2_vec = {
        {0.0, 0.0},
        {1.0, 1.0},
        {3.0, 2.0},
        {-2.0, 3.0}
    };

    CostFunctionQuadratic quadratic_function;
    test_2d_function("CostFunctionQuadratic", &quadratic_function, x2_vec);

    CostFunctionRosenbrock rosenbrock_function;
    test_2d_function("CostFunctionRosenbrock", &rosenbrock_function, x2_vec);

    CostFunctionHimmelblau himmelblau_function;
    std::vector<Eigen::Vector2d> himmelblau_points = {
        {3.0, 2.0},
        {-2.805118, 3.131312},
        {-3.77931, -3.283186},
        {3.584428, -1.848126}
    };
    test_2d_function("CostFunctionHimmelblau", &himmelblau_function, himmelblau_points);

    CostFunctionMcCormick mccormick_function;
    test_2d_function("CostFunctionMcCormick", &mccormick_function, x2_vec);

    // ----- CostFunctionExample (scalar API) -----
    std::cout << "=== CostFunctionExample (scalar API) ===" << std::endl;
    CostFunctionExample example_function;
    std::vector<double> x_scalar = {0.0, 2.0, 4.0, 6.0};
    for (double x : x_scalar) {
        double f = example_function.calculateCostFunctionValue(x);
        double g = example_function.calculateGradient(x);
        double h = example_function.calculateHessian(x);
        std::cout << "x = " << x << ", f(x) = " << f << ", f'(x) = " << g << ", f''(x) = " << h << std::endl;
    }
    std::cout << std::endl;

    // ----- CostFunctionExample (vector API, 1D: x(0) as scalar) -----
    std::cout << "=== CostFunctionExample (vector API, 1D) ===" << std::endl;
    CostFunctionBase* pExample = &example_function;
    std::vector<Eigen::VectorXd> x1_vec = {
        Eigen::Vector<double, 1>(0.0),
        Eigen::Vector<double, 1>(2.0),
        Eigen::Vector<double, 1>(4.0),
        Eigen::Vector<double, 1>(6.0)
    };
    Eigen::VectorXd gradient1(1);
    Eigen::MatrixXd hessian1(1, 1);
    for (const auto& x : x1_vec) {
        double f = pExample->calculateCostFunctionValue(x);
        pExample->calculateGradient(x, gradient1);
        pExample->calculateHessian(x, hessian1);
        std::cout << "x = " << std::endl << x << std::endl
                  << "f(x) = " << f << std::endl
                  << "g(x) = " << std::endl << gradient1 << std::endl
                  << "H(x) = " << std::endl << hessian1 << std::endl << std::endl;
    }

    return 0;
}