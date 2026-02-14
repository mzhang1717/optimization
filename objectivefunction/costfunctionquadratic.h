#pragma once
#include "costfunctionbase.h"

class CostFunctionQuadratic : public CostFunctionBase {
public:
    CostFunctionQuadratic() {}
    ~CostFunctionQuadratic() {}

    double calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x){
        double x1 = x(0);
        double x2 = x(1);

        return 4*x1*x1 + x2*x2 + 1;

    }
    void calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient){
        double x1 = x(0);
        double x2 = x(1);
                
        gradient(0) = 8*x1;
        gradient(1) = 2*x2;
    }
    void calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian){
        hessian(0,0) = 8;
        hessian(0,1) = 0;
        hessian(1,0) = 0;
        hessian(1,1) = 2;
    }

};