#include <cmath>
#include "costfunctionbase.h"

class OptimizerBase {
public:
    OptimizerBase();
    OptimizerBase(CostFunctionBase& costfunction);
    ~OptimizerBase();

    double doLineSearch(double x, double gradient, double searchDirection);
    double calculateSearchDirection(double x);
    virtual void CalculateSearchDirection();
    virtual void CalculateStepSize();
    virtual bool CheckTerminateCriteria();
    virtual void Update();
    void Optimize();

    void showResults();

    double run();


    CostFunctionBase* p_cost_function_;
    double cost_value_;
    double cost_gradient_;
    double x;
 

    double mConverageceParameter;
    int maxWorkCounter;
    double initial_guess_;
    double step_size_;
    double search_direction_;
    //double linesearchCoefficient;
};