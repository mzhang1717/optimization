#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "costfunctionbase.h"

class CostFunctionHimmelblau : public CostFunctionBase {
public:
    CostFunctionHimmelblau() {}
    ~CostFunctionHimmelblau() {}

    double calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x);
    void calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient);
    void calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian);



} ;