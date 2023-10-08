#include <eigen3/Eigen/Dense>
#include "../optimizerbase.h"
#include "../optimizerdescent.h"
#include "../../objectivefunction/costfunctionhimmelblau.h"
#include "../../objectivefunction/costfunctionmccormick.h"
//#include "costfunctionquadratic.h"

int main () {
    //CostFunctionQuadratic myfunction;
    //CostFunctionHimmelblau myfunction;
    CostFunctionMcComick myfunction;

    //OptimizerBase myOptimizer(myfunction, Eigen::Vector2d{2.0,1.0}) ;
    
    OptimizerDescent myOptimizerDescent(myfunction, Eigen::Vector2d{2.0,1.0}) ;
    OptimizerBase& myOptimizer{myOptimizerDescent};

    myOptimizer.optimize();

    myOptimizer.setInitialGuess(Eigen::Vector2d{-2,3.5});
    myOptimizer.optimize();

    myOptimizer.setInitialGuess(Eigen::Vector2d{-4,-3.5});
    myOptimizer.optimize();

    myOptimizer.setInitialGuess(Eigen::Vector2d{4,-2.5});
    myOptimizer.optimize();

    return 0;
}