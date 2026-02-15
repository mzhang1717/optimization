#include "costfunctionconvex3d.h"

double CostFunctionConvex3D::calculateCostFunctionValue(const Eigen::Ref<const Eigen::VectorXd>& x) {
    return x(0) * x(0) + x(1) * x(1) + x(2) * x(2);
}

void CostFunctionConvex3D::calculateGradient(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> gradient) {
    gradient(0) = 2 * x(0);
    gradient(1) = 2 * x(1);
    gradient(2) = 2 * x(2);
}

void CostFunctionConvex3D::calculateHessian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> hessian) {
    hessian.setZero();
    hessian(0, 0) = hessian(1, 1) = hessian(2, 2) = 2;
}
