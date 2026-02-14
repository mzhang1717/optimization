#include <iostream>
#include "optimizerbfgs.h"

OptimizerBFGS::OptimizerBFGS(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini):
OptimizerBase{costfunction, x_ini} {

    hessian_inverse_.resize(x_ini.rows(),x_ini.rows());

    gradient_epsilon_ = 0.000001;
    initial_step_size_ = 1.0;
    shrink_factor_ = 0.9;
    slope_factor_ = 0.0001;

    curve_factor_ = 0.9;

    assert(curve_factor_ > slope_factor_);
}

void OptimizerBFGS::calculateSearchDirection(){
    search_direction_ = - hessian_inverse_ * gradient_;
}

void OptimizerBFGS::initialUpdate(){
    OptimizerBase::initialUpdate();
    //ptr_cost_function_->calculateHessian(x_, hessian_);
    int n =  gradient_.rows();

    //hessian_inverse_ = gradient_.norm() * Eigen::MatrixXd::Identity(n,n);
    hessian_inverse_ = Eigen::MatrixXd::Identity(n,n);
    //std::cout << "H_0 = "  << std::endl << hessian_inverse_ << std::endl;
}

void OptimizerBFGS::update(){

    Eigen::VectorXd x_previous_ = x_;
    Eigen::VectorXd gradient_previous = gradient_;

    OptimizerBase::update();

    

    //s_k = x_k+1 - x_k = alpha_k * p_k
    Eigen::VectorXd s = x_ - x_previous_;

    //y_k = g_k+1 - g_k
    Eigen::VectorXd y = gradient_ - gradient_previous;

    // rho = 1/(y.T*s); skip update if y'*s not sufficiently positive (safeguard)
    const double y_dot_s = y.dot(s);
    const double curvature_epsilon = 1e-10;
    if (y_dot_s <= curvature_epsilon) {
        return;  // keep current hessian_inverse_, avoid division by zero / indefinite update
    }
    double rho = 1 / y_dot_s;

    // H = (I - rho*s*y.T)*H*(I - rho*y*s.T) + rho*s*s.T
    Eigen::MatrixXd H = hessian_inverse_;

    int n = H.rows();

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n,n);
    
    hessian_inverse_ = (I - rho*s*y.transpose())*H*(I - rho*y*s.transpose()) +
        rho*s*s.transpose();

}

void OptimizerBFGS::backtrackingLineSearch(){
    step_size_ = initial_step_size_;

    double directional_derivative = gradient_.transpose()*search_direction_;

    Eigen::VectorXd gradient_next = gradient_;
    ptr_cost_function_->calculateGradient(x_ + step_size_ * search_direction_, gradient_next);

    double function_value_next = ptr_cost_function_->calculateCostFunctionValue(x_ + step_size_ * search_direction_);
    const double min_step_size = 1e-14;

    int num_linesearch = 0;
     //shrink step size until f(x + step size * p) <= f(x) + step size*slope_factor_*p
    while (( function_value_next > (function_value_ + slope_factor_ * step_size_ * directional_derivative) ||
            gradient_next.transpose()*search_direction_ < curve_factor_* directional_derivative) 
            && num_linesearch <= max_linesearch_ && step_size_ >= min_step_size) {
                
                step_size_ *= shrink_factor_;

                ptr_cost_function_->calculateGradient(x_ + step_size_ * search_direction_, gradient_next);
                function_value_next = ptr_cost_function_->calculateCostFunctionValue(x_ + step_size_ * search_direction_);

                
                num_linesearch++;
    }

    if (step_size_ < min_step_size) {
        std::cout << "At " << number_iterations << "th iteration, step_size_ = "  << step_size_ << std::endl; 
    }
       
}