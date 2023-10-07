#include "optimizernewton.h"

void OptimizerNewton::CalculateSearchDirection()
{
    //calculate gradient g = pFun.calDeritive;
    //calculate hessian h = pFun.calHessian;
    //direction = -g/h

}

void OptimizerNewton::CalculateStepSize()
{
    //linesearch 
    // calculate f_k+1 = pFun.calValue(x + alpha * searchDirection)
    // calculate f_k = pFun.calValue(x)
    // alpha *= beta until f_k+1 - f_k <= mu * alpha * g * searchDirection
}

bool OptimizerNewton::CheckTerminateCriteria()
{
    // option 1: |g_k| <= eps_g
    // option 2: |f_k+1 - f_k| <= eps_a + eps_r * |f_k|

    return false;
}


// needed membervariables
// double stepsize
// double searchDirection
//

void OptimizerNewton::Update()
{
    cost_value_ = p_cost_function_->CalculateCostFunctionValue(x);
    cost_gradient_ = p_cost_function_->CalculateGradient(x);
    cost_hessian_ = p_cost_function_->CalculateHessian(x);
}
