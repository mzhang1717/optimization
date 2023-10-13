#pragma once

#include <eigen3/Eigen/Dense>
#include "../objectivefunction/costfunctionbase.h"
#include "optimizerbase.h"

class OptimizerDescent : public OptimizerBase {
public:
    OptimizerDescent(){}
    OptimizerDescent(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini);
    ~OptimizerDescent() {}

    //void backtrackingLineSearch();
    void calculateSearchDirection();
    //bool isTerminationReady();
    //void update();
    //void initialUpdate();
};