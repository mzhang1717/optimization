#include <cmath>
#include "costfunctionbase.h"

class OptimizerBase {
public:
    OptimizerBase();
    OptimizerBase(CostFunctionBase& costfunction);
    ~OptimizerBase();

    double doLineSearch(double x, double gradient, double searchDirection);
    double calculateSearchDirection(double x);

    void showResults();

    double run();

private:
    CostFunctionBase* pFunction;
    double mConverageceParameter;
    int maxWorkCounter;
    double initialGuess;
    //double linesearchCoefficient;
};