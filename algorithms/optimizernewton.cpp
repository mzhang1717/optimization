#include "optimizernewton.h"

OptimizerNewton::OptimizerNewton(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini):
OptimizerBase{costfunction, x_ini} {
    hessian_.resize(x_ini.rows(),x_ini.rows());
}


// void OptimizerNewton::backtrackingLineSearch(){

// }

void OptimizerNewton::calculateSearchDirection(){
    search_direction_ = -hessian_.inverse() * gradient_;
}

bool OptimizerNewton::isTerminationReady(){
    if (gradient_.norm() <= gradient_epsilon_ || number_iterations >= max_iterations_ ){
        return true;
    }
    else {
        return false;
    }
}

void OptimizerNewton::update(){
    OptimizerBase::update();
    ptr_cost_function_->calculateHessian(x_, hessian_);
}

void OptimizerNewton::initialUpdate(){
    OptimizerBase::initialUpdate();
    ptr_cost_function_->calculateHessian(x_, hessian_);

}