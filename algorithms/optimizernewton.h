#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "optimizerbase.h"

class OptimizerNewton : public OptimizerBase {
public:
    OptimizerNewton() {}
    OptimizerNewton(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini);
    ~OptimizerNewton() {}

    //virtual void backtrackingLineSearch();
    virtual void calculateSearchDirection();
    virtual bool isTerminationReady();
    virtual void update();
    virtual void initialUpdate();
    bool isPositiveDefiniteMatrix(const Eigen::Ref<const Eigen::MatrixXd>& A);

private:
    Eigen::MatrixXd hessian_;
};