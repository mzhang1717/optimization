#include "optimizerdescent.h"

OptimizerDescent::OptimizerDescent(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini)
    : OptimizerBase{costfunction, x_ini} {
    gradient_epsilon_ = 0.000001;
    initial_step_size_ = 1.0;
    shrink_factor_ = 0.5;
    slope_factor_ = 0.5;
}

void OptimizerDescent::calculateSearchDirection() {
    search_direction_ = -gradient_;
}

