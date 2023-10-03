#include "optimizerbase.h"
#include <iostream>

OptimizerBase::OptimizerBase() {

}

OptimizerBase::OptimizerBase(CostFunctionBase& costfunction) {
    pFunction = &costfunction;
    mConverageceParameter = 0.000001;
    initialGuess = 120.0;
    maxWorkCounter = 500;
}

OptimizerBase::~OptimizerBase() {

}

double OptimizerBase::doLineSearch(double x, double gradient, double searchDirection)
{
    double alpha = 0.7;
    double discountRatio = 0.5;
    double mu = 0.0001;

    while (pFunction->calculateCostFunctionValue(x + alpha*searchDirection) > 
            (pFunction->calculateCostFunctionValue(x) + mu * alpha * gradient*searchDirection)) {
                std::cout << "alpha = " << alpha << std::endl;
                alpha *= discountRatio;
    }

    std::cout << "alpha = " << alpha << std::endl;

    return alpha;
}

double OptimizerBase::calculateSearchDirection(double x)
{
    double hessian = pFunction->calculateHessian(x);
    double gradient = pFunction->calculateGradient(x);

    return - gradient/hessian;
}

void OptimizerBase::Optimize()
{
    while(!CheckTerminateCriteria()) {
        CalculateSearchDirection();
        CalculateStepSize;
        Update();
    }
}

void OptimizerBase::showResults()
{
    double x = run();
    double f_val = pFunction->calculateCostFunctionValue(x);

    std::cout << "x_min = " << x << ", " << "f(x_min) = " << f_val << std::endl;
}

double OptimizerBase::run() {
    double x = initialGuess;
    double gradieent = pFunction->calculateGradient(x);
    int workCounter = 0;

    //std::cout << "maxworkCounter = " << maxWorkCounter << std::endl;
    std::cout << "x_0 = "  << x << ", " 
            << "f_0 = " << pFunction->calculateCostFunctionValue(x) << ", " 
            << "g_0 = " << gradieent << std::endl;

    //std::cout << gradieent << ", " << abs(gradieent) << std::endl;

    while ((std::abs(gradieent) > mConverageceParameter) && (workCounter < maxWorkCounter)) {
        double searchdirection = calculateSearchDirection(x);
        double stepsize = doLineSearch(x, gradieent, searchdirection);

        //std::cout << "stepsize = " << stepsize << ", searchdirection= " << searchdirection << std::endl;

        x += (stepsize * searchdirection);
        workCounter += 1;

        gradieent = pFunction->calculateGradient(x);

        std::cout << "x_" << workCounter << "= " << x << ", " 
            << "f_" << workCounter << "=" << pFunction->calculateCostFunctionValue(x) << ", " 
            << "g_" << workCounter << "=" << gradieent<< std::endl;

        //std::cout << gradieent << ", " << std::abs(gradieent) << std::endl;

        
    }    

    return x;
}