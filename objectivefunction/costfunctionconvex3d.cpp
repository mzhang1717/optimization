#include "costfunctionconvex3d.h"

double CostFunctionConvex3D::calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x) {
    double x1 = x(0);
    double x2 = x(1);
    double x3 = x(2);
    return x1 * x1 + x2 * x2 + x3 * x3;
}

void CostFunctionConvex3D::calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient) {
    gradient(0) = 2 * x(0);
    gradient(1) = 2 * x(1);
    gradient(2) = 2 * x(2);
}

void CostFunctionConvex3D::calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian) {
    hessian.setZero();
    hessian(0, 0) = 2;
    hessian(1, 1) = 2;
    hessian(2, 2) = 2;
}
