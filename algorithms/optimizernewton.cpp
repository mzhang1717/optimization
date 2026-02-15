#include <iostream>
#include "optimizernewton.h"

OptimizerNewton::OptimizerNewton(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini)
    : OptimizerBase{costfunction, x_ini} {
    hessian_.resize(x_ini.rows(), x_ini.rows());
    gradient_epsilon_ = 0.000001;
    initial_step_size_ = 1.0;
    shrink_factor_ = 0.9;
    slope_factor_ = 0.0001;
}

void OptimizerNewton::calculateSearchDirection() {
    if (isPositiveDefiniteMatrix(hessian_)) {
        // Solve H * d = -g via LDLT (numerically more stable than forming H^{-1}).
        search_direction_ = -hessian_.ldlt().solve(gradient_);
    } else {
        std::cout << "At " << number_iterations << "th iteration: Hessian is NOT positive definite!" << std::endl;
        std::cout << "Hessian = " << std::endl << hessian_ << std::endl;
        search_direction_ = -gradient_;  // Fallback to steepest descent.
    }
}

void OptimizerNewton::update() {
    OptimizerBase::update();
    ptr_cost_function_->calculateHessian(x_, hessian_);
}

void OptimizerNewton::initialUpdate() {
    OptimizerBase::initialUpdate();
    ptr_cost_function_->calculateHessian(x_, hessian_);
}

bool OptimizerNewton::isPositiveDefiniteMatrix(const Eigen::Ref<const Eigen::MatrixXd>& A) {
    Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
    return ldlt.isPositive();
}