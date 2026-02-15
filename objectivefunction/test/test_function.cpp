#include <vector>
#include <iostream>
//#include <matplot/matplot.h>
#include <eigen3/Eigen/Dense>
#include "../costfunctionexample.h"
#include "../costfunctionquadratic.h"
#include "../costfunctionhimmelblau.h"
#include "../costfunctionmccormick.h"
#include "../costfunctionrosenbrock.h"
#include "../costfunctionconvex3d.h"


int main() {

    // ----- Test CostFunctionConvex3D (3 variables: f(x) = x1^2 + x2^2 + x3^2) -----
    std::cout << "=== CostFunctionConvex3D ===" << std::endl;
    CostFunctionConvex3D convex3d_function;
    CostFunctionBase* pConvex3d = &convex3d_function;

    std::vector<Eigen::Vector3d> x3_vec;
    x3_vec.push_back(Eigen::Vector3d{0.0, 0.0, 0.0});
    x3_vec.push_back(Eigen::Vector3d{1.0, 2.0, 3.0});
    x3_vec.push_back(Eigen::Vector3d{-1.0, 0.5, -2.0});

    Eigen::Vector3d gradient3;
    Eigen::Matrix3d hessian3;

    for (const auto& x : x3_vec) {
        double f = pConvex3d->calculateCostFunctionValue(x);
        pConvex3d->calculateGradient(x, gradient3);
        pConvex3d->calculateHessian(x, hessian3);

        std::cout << "x = " << std::endl << x << std::endl
                  << "f(x) = " << std::endl << f << std::endl
                  << "g(x) = " << std::endl << gradient3 << std::endl
                  << "H(x) = " << std::endl << hessian3 << std::endl << std::endl;
    }

    // ----- Test 2D cost function (e.g. McCormick) -----
    std::cout << "=== CostFunctionMcCormick (2D) ===" << std::endl;
    std::vector<Eigen::Vector2d> x_vec;
    x_vec.push_back(Eigen::Vector2d{3.0, 2.0});
    x_vec.push_back(Eigen::Vector2d{-2.805118, 3.131312});
    x_vec.push_back(Eigen::Vector2d{-3.77931, -3.283186});
    x_vec.push_back(Eigen::Vector2d{3.584428, -1.848126});

    double function_value;
    Eigen::Vector2d gradient;
    Eigen::Matrix2d hessian;

    // CostFunctionExample / CostFunctionQuadratic / CostFunctionHimmelblau / CostFunctionRosenbrock / CostFunctionMcCormick
    CostFunctionMcCormick myfunction;
    CostFunctionBase* pFunction = &myfunction;

    for (const auto& x : x_vec) {
        function_value = pFunction->calculateCostFunctionValue(x);
        pFunction->calculateGradient(x, gradient);
        pFunction->calculateHessian(x, hessian);

        std::cout << "x = " << std::endl << x << std::endl
                  << "f(x) = " << std::endl << function_value << std::endl
                  << "g(x) = " << std::endl << gradient << std::endl
                  << "H(x) = " << std::endl << hessian << std::endl << std::endl;
    }


    // std::vector<double> x_list {0, 2.0, 4.0, 6.0};
//    int num_points =  81;
/*     std::vector<double> x_list(num_points);
    std::vector<double> value_list(num_points);
    std::vector<double> derivative_list(num_points);
    std::vector<double> hessian_list(num_points); */

    //std::cout << "x - Value - Gradient - Hessian" << std::endl ;

//     double x = -1.0 ;
//     double step = 0.1;

//     for (int i = 0; i < num_points; i++) {
// /*         x_list[i] = x;
//         value_list[i] = myfunction.calculateCostFunctionValue(x);
//         derivative_list[i] = myfunction.calculateGradient(x);
//         hessian_list[i] = myfunction.calculateHessian(x); */

        

//          std::cout << x <<  "; " 
//                      << pFunction->calculateCostFunctionValue(x) << "; " 
//                     << pFunction->calculateGradient(x) << "; " 
//                     << pFunction->calculateHessian(x) << std::endl ;  

//         x += step;
//     }

/*     matplot::plot(x_list, value_list);
    matplot::hold(on);

    matplot::show(); */
    
    return 0;
}