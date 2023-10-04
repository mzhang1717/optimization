#pragma once
#include <eigen3/Eigen/Dense>

class CostFunctionBase {
public:
    CostFunctionBase(){}
    ~CostFunctionBase(){}

    virtual double calculateCostFunctionValue(double x) { return 0;}
    virtual double calculateGradient(double x){return 0;}
    virtual double calculateHessian(double x){return 0;}

    virtual double calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x) { return 0;}
    virtual void calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient){return;}
    virtual void calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian){return;}
};