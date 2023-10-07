#include "optimizerbase.h"
#include <iostream>

OptimizerBase::OptimizerBase() {

}

OptimizerBase::OptimizerBase(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini) 
:ptr_cost_function_(&costfunction ) {
    initial_guess_ = x_ini;
    gradient_ = x_ini; // to set its size to the same as x
    //std::cout << "initial_guess_ = " << initial_guess_ << "---------------"<< std::endl;
    max_iterations_ = 500;
    gradient_epsilon_ = 0.000001;
    initial_step_size_ = 1.0;
    shrink_factor_ = 0.5;
    slope_factor_ =0.5;
}

OptimizerBase::~OptimizerBase() {
    //delete ptr_cost_function_;
}

void OptimizerBase::setInitialGuess(const Eigen::Ref<const Eigen::VectorXd>& x_ini) {
    initial_guess_ = x_ini;
}

void OptimizerBase::optimize()
{
    initialUpdate();

    while(!isTerminationReady()) {
        calculateSearchDirection();
        backtrackingLineSearch();
        update();
    }
    
    showResults();
}

bool OptimizerBase::isTerminationReady() {

    if (gradient_.norm() <= gradient_epsilon_ || number_iterations >= max_iterations_ ){
        return true;
    }
    else {
        return false;
    }
}

void OptimizerBase::update() {
    number_iterations++;
    //std::cout << "num_iterations = " << number_iterations << std::endl;

    x_ += step_size_*search_direction_;
    //std::cout << "x = " << x_ << std::endl;

    function_value_ = ptr_cost_function_->calculateCostFunctionValue(x_);
    //std::cout << "f(x) = " << function_value_ << std::endl;

    ptr_cost_function_->calculateGradient(x_, gradient_);
    //std::cout << "g(x) = " << gradient_ << "----------------" <<std::endl; 
    

}

void OptimizerBase::initialUpdate(){

    number_iterations = 0;
    //std::cout << "num_iterations = " << number_iterations << std::endl;

    x_ = initial_guess_;
    //std::cout << "x = " << x_ << std::endl;

    function_value_ = ptr_cost_function_->calculateCostFunctionValue(x_);
    //std::cout << "f(x) = " << function_value_ << std::endl;

    ptr_cost_function_->calculateGradient(x_, gradient_);
    //std::cout << "g(x) = " << gradient_ << "----------------" <<std::endl;    
}

void OptimizerBase::calculateSearchDirection()
{
    search_direction_ = -gradient_;
}

void OptimizerBase::backtrackingLineSearch(){
    step_size_ = initial_step_size_;

    double gradient_norm = gradient_.norm();
    double gradient_square = gradient_norm * gradient_norm;

     //shrink step size until f(x - step size * p) <= f(x) - step size*slope_factor_*p
    while (ptr_cost_function_->calculateCostFunctionValue(x_ - step_size_ * gradient_) 
            > function_value_ - slope_factor_ * step_size_ * gradient_square) {

                step_size_ *= shrink_factor_;
    }
}

void OptimizerBase::showResults()
{
    std::cout << "x_min = " << x_ << std::endl
     << "f(x_min) = " << function_value_ << std::endl
     << "g(x_min) = " << gradient_ << std::endl
     << "num_iterations = " << number_iterations << std::endl << std::endl;
}