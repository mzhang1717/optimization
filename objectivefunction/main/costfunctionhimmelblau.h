#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "costfunctionbase.h"

class CostFunctionHimmelblau : public CostFunctionBase {
public:
    CostFunctionHimmelblau() {}
    ~CostFunctionHimmelblau() {}

    double calculateCostFunctionValue(Eigen::Vector2d x);
    Eigen::Vector2d calculateGradient(Eigen::Vector2d x);
    Eigen::Matrix2d calculateHessian(Eigen::Vector2d x);



} ;