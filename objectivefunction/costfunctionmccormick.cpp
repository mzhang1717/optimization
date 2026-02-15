#include <cmath>
#include "costfunctionmccormick.h"

double CostFunctionMcCormick::calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x) {
    double x1 = x(0);
    double x2 = x(1);
    return std::sin(x1 + x2) + (x1 - x2) * (x1 - x2) - 1.5 * x1 + 2.5 * x2 + 1;
}

void CostFunctionMcCormick::calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient) {
    double x1 = x(0);
    double x2 = x(1);
    double cos_sum = std::cos(x1 + x2);
    gradient(0) = cos_sum + 2 * (x1 - x2) - 1.5;
    gradient(1) = cos_sum - 2 * (x1 - x2) + 2.5;
}

void CostFunctionMcCormick::calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian) {
    double x1 = x(0);
    double x2 = x(1);
    double sin_sum = std::sin(x1 + x2);
    hessian(0, 0) = -sin_sum + 2;
    hessian(0, 1) = -sin_sum - 2;
    hessian(1, 0) = -sin_sum - 2;
    hessian(1, 1) = -sin_sum + 2;
}
