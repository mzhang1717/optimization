#pragma once

#include "costfunctionbase.h"

// This is a example of a cost function that is used to test the optimizer.
// Note that this is a scalar function, not a vector function.
// The optimizer shall not use this function to test the optimizer. 


class CostFunctionExample : public CostFunctionBase {
public:
    CostFunctionExample(){}
    ~CostFunctionExample(){}

    double calculateCostFunctionValue(double x);
    double calculateGradient(double x);
    double calculateHessian(double x);
};