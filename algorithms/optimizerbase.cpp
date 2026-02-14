#include <iostream>
#include <cmath>
#include "optimizerbase.h"


OptimizerBase::OptimizerBase() {

}

OptimizerBase::OptimizerBase(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini) 
:ptr_cost_function_{&costfunction }, initial_guess_{x_ini} {
    gradient_.resize(x_ini.rows(), x_ini.cols()); // to set its size to the same as x
    //std::cout << "initial_guess_ = " << initial_guess_ << "---------------"<< std::endl;
    max_iterations_ = 5000;
    max_linesearch_ = 50;


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

        if (std::isnan(function_value_)) {
            std::cerr << "*** NaN *** at step " << number_iterations << std::endl;
            break;
        }
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

    x_ = initial_guess_;
    function_value_ = ptr_cost_function_->calculateCostFunctionValue(x_);
    ptr_cost_function_->calculateGradient(x_, gradient_);

    // Ensure search_direction_ has correct size before first calculateSearchDirection()
    search_direction_.resizeLike(gradient_);
}

void OptimizerBase::backtrackingLineSearch(){
    step_size_ = initial_step_size_;

    double gradient_square_negative = gradient_.transpose()*search_direction_;

    const double min_step_size = 1e-14;
    int num_linesearch = 0;
    // Shrink step until Armijo: f(x + step*p) <= f(x) + step*slope_factor_*gradient'*p
    while (ptr_cost_function_->calculateCostFunctionValue(x_ + step_size_ * search_direction_)
            > (function_value_ + slope_factor_ * step_size_ * gradient_square_negative)) {

        step_size_ *= shrink_factor_;
        num_linesearch++;
        if (num_linesearch >= max_linesearch_ || step_size_ < min_step_size) {
            break;
        }
    }
}

void OptimizerBase::showResults() const
{
    std::cout << "x_min = " << std::endl << x_ << std::endl
     << "f(x_min) = " << function_value_ << std::endl
     << "g(x_min) = " <<  std::endl << gradient_  << std::endl
     << "num_iterations = " << number_iterations << std::endl << std::endl;
}