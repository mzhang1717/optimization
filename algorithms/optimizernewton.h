#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "optimizerbase.h"

class OptimizerNewton : public OptimizerBase {
public:
    OptimizerNewton() {}
    OptimizerNewton(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini);
    ~OptimizerNewton() {}

    // void backtrackingLineSearch();
    void calculateSearchDirection();
    //bool isTerminationReady();
    void update();
    void initialUpdate();
    bool isPositiveDefiniteMatrix(const Eigen::Ref<const Eigen::MatrixXd>& A);

private:
    Eigen::MatrixXd hessian_;
};