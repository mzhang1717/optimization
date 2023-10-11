#include <iostream>
#include <eigen3/Eigen/Dense>
#include "../optimizerbase.h"
#include "../optimizerdescent.h"
#include "../optimizernewton.h"
#include "../optimizerbfgs.h"
#include "../../objectivefunction/costfunctionhimmelblau.h"
#include "../../objectivefunction/costfunctionmccormick.h"
#include "../../objectivefunction/costfunctionquadratic.h"
#include "../../objectivefunction/costfunctionrosenbrock.h"
//#include "costfunctionquadratic.h"

int main () {
    //CostFunctionQuadratic myfunction;
    //CostFunctionHimmelblau myfunction;
    //CostFunctionMcComick myfunction;
    //CostFunctionQuadratic myfunction;
    CostFunctionRosenbrock myfunction;

    //OptimizerBase myOptimizerBase(myfunction, Eigen::Vector2d{2.0,1.0}) ;
    //OptimizerDescent myOptimizerDescent(myfunction, Eigen::Vector2d{2.0,1.0}) ;
    OptimizerNewton myOptimizerNewton(myfunction, Eigen::Vector2d{2.1,1.0}) ;

    std::cout <<"------- Newton method -----" << std::endl;

    OptimizerBase* ptrOptimizer{&myOptimizerNewton};

    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-4,5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-1,-5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{6,-3});
    ptrOptimizer->optimize();

    std::cout <<"------- BFGS method -----" << std::endl;
    //OptimizerDescent myOptimizerDescent(myfunction, Eigen::Vector2d{2.0,1.0}) ;
    //ptrOptimizer = &myOptimizerDescent;

    OptimizerBFGS myOPtimizerBFGS(myfunction, Eigen::Vector2d{-1.2,1.0}) ;
    ptrOptimizer = &myOPtimizerBFGS;

    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-4,5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{-1,-5});
    ptrOptimizer->optimize();

    ptrOptimizer->setInitialGuess(Eigen::Vector2d{6,-3});
    ptrOptimizer->optimize();


    return 0;
}