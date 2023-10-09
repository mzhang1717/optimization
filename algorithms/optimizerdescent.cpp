#include "optimizerdescent.h"

OptimizerDescent::OptimizerDescent(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini):
OptimizerBase{costfunction, x_ini} {

}

// void OptimizerDescent::backtrackingLineSearch(){

// }

void OptimizerDescent::calculateSearchDirection(){
    search_direction_ = -gradient_;
}

bool OptimizerDescent::isTerminationReady(){
    if (gradient_.norm() <= gradient_epsilon_ || number_iterations >= max_iterations_ ){
        return true;
    }
    else {
        return false;
    }
}

// void OptimizerDescent::update(){

// }

// void OptimizerDescent::initialUpdate(){

// }