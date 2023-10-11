#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "optimizerbase.h"

class OptimizerBFGS : public OptimizerBase {
public:
    OptimizerBFGS() {}
    OptimizerBFGS(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini);
    ~OptimizerBFGS() {}


    void calculateSearchDirection();
    bool isTerminationReady();
    void update();
    void initialUpdate();
    void backtrackingLineSearch();
    //bool isPositiveDefiniteMatrix(const Eigen::Ref<const Eigen::MatrixXd>& A);

private:
    //Eigen::MatrixXd hessian_;
    Eigen::MatrixXd hessian_inverse_;
    double curve_factor_;
};