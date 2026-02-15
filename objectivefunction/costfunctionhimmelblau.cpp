#include "costfunctionhimmelblau.h"

double CostFunctionHimmelblau::calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x) {
    double x1 = x(0);
    double x2 = x(1);
    return (x1 * x1 + x2 - 11) * (x1 * x1 + x2 - 11) + (x1 + x2 * x2 - 7) * (x1 + x2 * x2 - 7);
}

void CostFunctionHimmelblau::calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient) {
    double x1 = x(0);
    double x2 = x(1);
    gradient(0) = 4 * x1 * x1 * x1 + 4 * x1 * x2 + 2 * x2 * x2 - 42 * x1 - 14;
    gradient(1) = 4 * x2 * x2 * x2 + 4 * x1 * x2 + 2 * x1 * x1 - 26 * x2 - 22;
}

void CostFunctionHimmelblau::calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian) {
    double x1 = x(0);
    double x2 = x(1);
    double h11 = 12 * x1 * x1 + 4 * x2 - 42;
    double h12 = 4 * x1 + 4 * x2;
    double h22 = 12 * x2 * x2 + 4 * x1 - 26;
    hessian(0, 0) = h11;
    hessian(0, 1) = h12;
    hessian(1, 0) = h12;
    hessian(1, 1) = h22;
}