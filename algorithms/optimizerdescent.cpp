#include "optimizerdescent.h"

OptimizerDescent::OptimizerDescent(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini):
OptimizerBase{costfunction, x_ini} {

}

void OptimizerDescent::backtrackingLineSearch(){
    step_size_ = initial_step_size_;

    double gradient_norm = gradient_.norm();
    double gradient_square = gradient_norm * gradient_norm;

     //shrink step size until f(x - step size * p) <= f(x) - step size*slope_factor_*p
    while (ptr_cost_function_->calculateCostFunctionValue(x_ - step_size_ * gradient_) 
            > function_value_ - slope_factor_ * step_size_ * gradient_square) {

                step_size_ *= shrink_factor_;
    }    
}

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