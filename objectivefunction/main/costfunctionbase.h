#pragma once
#include <eigen3/Eigen/Dense>

class CostFunctionBase {
public:
    CostFunctionBase(){}
    ~CostFunctionBase(){}

    virtual double calculateCostFunctionValue(double x) { return 0;}
    virtual double calculateGradient(double x){return 0;}
    virtual double calculateHessian(double x){return 0;}

    virtual double calculateCostFunctionValue(Eigen::VectorXd x) { return 0;}
    virtual Eigen::VectorXd calculateGradient(Eigen::VectorXd x){return Eigen::VectorXd::Zero(x.rows());}
    virtual Eigen::MatrixXd calculateHessian(Eigen::VectorXd x){return Eigen::MatrixXd::Identity(x.rows(), x.rows());}
};