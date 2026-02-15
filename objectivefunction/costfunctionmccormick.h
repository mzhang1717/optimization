#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "costfunctionbase.h"

class CostFunctionMcCormick : public CostFunctionBase {
public:
    CostFunctionMcCormick(){}
    ~CostFunctionMcCormick(){}

    virtual double calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x);
    virtual void calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient);
    virtual void calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian);
};