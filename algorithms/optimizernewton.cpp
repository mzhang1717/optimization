#include <iostream>
#include "optimizernewton.h"

OptimizerNewton::OptimizerNewton(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini):
OptimizerBase{costfunction, x_ini} {
    hessian_.resize(x_ini.rows(),x_ini.rows());
}


// void OptimizerNewton::backtrackingLineSearch(){

// }

void OptimizerNewton::calculateSearchDirection(){
    if(isPositiveDefiniteMatrix(hessian_)){
        search_direction_ = -hessian_.inverse() * gradient_;
    }
    else {
        std::cout << "At " << number_iterations <<  "th iteration: Hessian is NOT positive definite!" << std::endl;
        
        std::cout << "Hessian = " << std::endl << hessian_ << std::endl;
        
        search_direction_ = -gradient_;
    } 
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

bool OptimizerNewton::isPositiveDefiniteMatrix(const Eigen::Ref<const Eigen::MatrixXd>& A) {
    // if (A.rows() != A.cols()) {
    //     std::cerr << "Matrix is not square!" << std::endl;
    //     return false;
    // }

    int n = A.rows();
    bool bPositive = true;

    for (int i = 1; i<=n; i++){
        if (A.block(0,0,i,i).determinant() <= 0) {
            bPositive = false;
            break;
        }
    }

    return bPositive;
}