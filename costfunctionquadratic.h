#include <cmath>
#pragma once
#include "costfunctionbase.h"

class CostFunctionQuadratic : public CostFunctionBase {
public:
    CostFunctionQuadratic() {}
    ~CostFunctionQuadratic() {}

    double calculateCostFunctionValue(double x) { return x*x + 2*x +1;}
    double calculateGradient(double x){return 2*x + 2;}
    double calculateHessian(double x){return 2;}

};