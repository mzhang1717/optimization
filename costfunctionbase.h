#pragma once

class CostFunctionBase {
public:
    CostFunctionBase(){}
    ~CostFunctionBase(){}

    virtual double calculateCostFunctionValue(double x) { return 0;}
    virtual double calculateGradient(double x){return 0;}
    virtual double calculateHessian(double x){return 0;}
};