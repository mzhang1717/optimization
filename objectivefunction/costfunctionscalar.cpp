#include "costfunctionscalar.h"
#include <cmath>

// Polynomial f(x) = x^8 - 27*x^7 + 288*x^6 - 1512*x^5 + 3888*x^4 - 3888*x^3 (and derivatives).

double CostFunctionScalar::calculateCostFunctionValue(double x) {
    return std::pow(x, 8) - 27 * std::pow(x, 7) + 288 * std::pow(x, 6) - 1512 * std::pow(x, 5)
           + 3888 * std::pow(x, 4) - 3888 * std::pow(x, 3);
}

double CostFunctionScalar::calculateGradient(double x) {
    return 8 * std::pow(x, 7) - 7 * 27 * std::pow(x, 6) + 6 * 288 * std::pow(x, 5)
           - 5 * 1512 * std::pow(x, 4) + 4 * 3888 * std::pow(x, 3) - 3 * 3888 * std::pow(x, 2);
}

double CostFunctionScalar::calculateHessian(double x) {
    return 56 * std::pow(x, 6) - 42 * 27 * std::pow(x, 5) + 30 * 288 * std::pow(x, 4)
           - 20 * 1512 * std::pow(x, 3) + 12 * 3888 * std::pow(x, 2) - 6 * 3888 * x;
}

double CostFunctionScalar::calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x) {
    return calculateCostFunctionValue(x(0));
}

void CostFunctionScalar::calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient) {
    gradient(0) = calculateGradient(x(0));
}

void CostFunctionScalar::calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian) {
    hessian(0, 0) = calculateHessian(x(0));
}
