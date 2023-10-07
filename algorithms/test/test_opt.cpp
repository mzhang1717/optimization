#include <eigen3/Eigen/Dense>
#include "../optimizerbase.h"
#include "../../objectivefunction/costfunctionhimmelblau.h"
//#include "costfunctionquadratic.h"

int main () {
    //CostFunctionQuadratic myfunction;
    CostFunctionHimmelblau myHimmelblau;
    OptimizerBase myOptimizer(myHimmelblau, Eigen::Vector2d{2.0,1.0}) ;

    myOptimizer.optimize();

    myOptimizer.setInitialGuess(Eigen::Vector2d{-2,3.5});
    myOptimizer.optimize();

    myOptimizer.setInitialGuess(Eigen::Vector2d{-4,-3.5});
    myOptimizer.optimize();

    myOptimizer.setInitialGuess(Eigen::Vector2d{4,-2.5});
    myOptimizer.optimize();

    return 0;
}