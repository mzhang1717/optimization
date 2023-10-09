#pragma once

#include <eigen3/Eigen/Dense>
#include "../objectivefunction/costfunctionbase.h"
#include "optimizerbase.h"

class OptimizerDescent : public OptimizerBase {
public:
    OptimizerDescent(){}
    OptimizerDescent(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini);
    ~OptimizerDescent() {}

    //virtual void backtrackingLineSearch();
    virtual void calculateSearchDirection();
    virtual bool isTerminationReady();
    //virtual void update();
    //virtual void initialUpdate();
};