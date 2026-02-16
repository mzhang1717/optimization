#include <iostream>
#include <cmath>
#include "optimizerbase.h"

OptimizerBase::OptimizerBase() {}

OptimizerBase::OptimizerBase(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini)
    : ptr_cost_function_{&costfunction}, initial_guess_{x_ini} {
    gradient_.resize(x_ini.rows(), x_ini.cols());
    max_iterations_ = 5000;
    max_linesearch_ = 50;
}

OptimizerBase::~OptimizerBase() {}

void OptimizerBase::setInitialGuess(const Eigen::Ref<const Eigen::VectorXd>& x_ini) {
    initial_guess_ = x_ini;
}

void OptimizerBase::optimize() {
    initialUpdate();
    while (!isTerminationReady()) {
        calculateSearchDirection();
        backtrackingLineSearch();
        update();
        if (std::isnan(function_value_)) {
            std::cerr << "*** NaN *** at step " << number_iterations_ << std::endl;
            break;
        }
    }
    showResults();
}

bool OptimizerBase::isTerminationReady() {
    return (gradient_.norm() <= gradient_epsilon_ || number_iterations_ >= max_iterations_);
}

void OptimizerBase::update() {
    number_iterations_++;
    x_ += step_size_ * search_direction_;
    function_value_ = ptr_cost_function_->calculateCostFunctionValue(x_);
    ptr_cost_function_->calculateGradient(x_, gradient_);
}

void OptimizerBase::initialUpdate() {
    number_iterations_ = 0;
    x_ = initial_guess_;
    function_value_ = ptr_cost_function_->calculateCostFunctionValue(x_);
    ptr_cost_function_->calculateGradient(x_, gradient_);
    search_direction_.resizeLike(gradient_);
}

void OptimizerBase::backtrackingLineSearch() {
    step_size_ = initial_step_size_;
    // Directional derivative: gradient' * search_direction_ (used in Armijo condition).
    double directional_derivative = gradient_.transpose() * search_direction_;

    const double min_step_size = 1e-14;
    int num_linesearch = 0;
    // Armijo: accept when f(x + step*p) <= f(x) + slope_factor_ * step * (gradient'*p).
    while (ptr_cost_function_->calculateCostFunctionValue(x_ + step_size_ * search_direction_)
           > (function_value_ + slope_factor_ * step_size_ * directional_derivative)) {
        step_size_ *= shrink_factor_;
        num_linesearch++;
        if (num_linesearch >= max_linesearch_ || step_size_ < min_step_size) {
            break;
        }
    }
}

void OptimizerBase::showResults() const {
    std::cout << "x_min = " << std::endl << x_ << std::endl
              << "f(x_min) = " << function_value_ << std::endl
              << "g(x_min) = " << std::endl << gradient_ << std::endl
              << "num_iterations = " << number_iterations_ << std::endl << std::endl;
}