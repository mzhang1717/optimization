#pragma once
#include <eigen3/Eigen/Dense>
#include "costfunctionbase.h"

class CostFunctionConvex3D : public CostFunctionBase {
public:
    CostFunctionConvex3D() {}
    ~CostFunctionConvex3D() {}

    virtual double calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x);
    virtual void calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient);
    virtual void calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian);
};
