#pragma once

#include "costfunctionbase.h"

class CostFunctionExample : public CostFunctionBase {
public:
    CostFunctionExample(){}
    ~CostFunctionExample(){}

    double calculateCostFunctionValue(double x);
    double calculateGradient(double x);
    double calculateHessian(double x);
};