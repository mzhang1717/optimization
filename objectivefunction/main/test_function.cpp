#include "costfunctionexample.h"
#include "costfunctionquadratic.h"
#include "costfunctionhimmelblau.h"
#include <vector>
#include <iostream>
//#include <matplot/matplot.h>
#include <eigen3/Eigen/Dense>

int main() {

    // Eigen::Matrix2d m;
    // m << 1, 2, 3, 4;
    // std::cout << m << std::endl;
    std::vector<Eigen::Vector2d> x_vec;
    x_vec.push_back(Eigen::Vector2d{3.0,2.0});
    x_vec.push_back(Eigen::Vector2d{-2.8,3.1});
    x_vec.push_back(Eigen::Vector2d{-3.8,-3.3});
    x_vec.push_back(Eigen::Vector2d{3.6,-1.8});



    
    //CostFunctionExample myfunction;
    //CostFunctionQuadratic qfunction;
    CostFunctionHimmelblau myfunction;

    CostFunctionBase* pFunction{&myfunction};

    for (auto x : x_vec) {
        std::cout << x << std::endl 
        << pFunction->calculateCostFunctionValue(x) << std::endl 
        << pFunction->calculateGradient(x) << std::endl
        << pFunction->calculateHessian(x) << std::endl << std::endl;


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
//         value_list[i] = myfuncion.calculateCostFunctionValue(x);
//         derivative_list[i] = myfuncion.calculateGradient(x);
//         hessian_list[i] = myfuncion.calculateHessian(x); */

        

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