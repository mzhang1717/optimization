#include "optimizerbase.h"
#include "costfunctionquadratic.h"

int main () {
    CostFunctionQuadratic myfunction;
    OptimizerBase myNewtonOptimizer(myfunction);

    myNewtonOptimizer.showResults();

    return 0;
}