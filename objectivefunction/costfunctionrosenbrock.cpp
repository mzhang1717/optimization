#include "costfunctionrosenbrock.h"

double CostFunctionRosenbrock::calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x){
    double x1 = x(0);
    double x2 = x(1);

    return (1 - x1)*(1 - x1) + 100*(x2 - x1*x1)*(x2 - x1*x1);
}
void CostFunctionRosenbrock::calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient){
    double x1 = x(0);
    double x2 = x(1);

    gradient(0) = 400*x1*x1*x1 - 400*x1*x2 + 2*x1 - 2;
    gradient(1) = 200*(x2 - x1*x1);
}
void CostFunctionRosenbrock::calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian){
    double x1 = x(0);
    double x2 = x(1);

    hessian(0,0) = 1200*x1*x1 - 400*x2 + 2;
    hessian(0,1) = -400*x1;
    hessian(1,0) = -400*x1;
    hessian(1,1) = 200;
}