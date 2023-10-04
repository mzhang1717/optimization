#include "costfunctionhimmelblau.h"

double CostFunctionHimmelblau::calculateCostFunctionValue(Eigen::Vector2d x) {
    double x1 = x(0);
    double x2 = x(1);

    return (x1*x1 + x2 - 11) * (x1*x1 + x2 -11) + (x1 + x2*x2 - 7)*(x1 + x2*x2 - 7);

}

Eigen::Vector2d CostFunctionHimmelblau::calculateGradient(Eigen::Vector2d x) {
    double x1 = x(0);
    double x2 = x(1);

    double df_dx1 = 4*x1*x1*x1 + 4*x1*x2 + 2*x2*x2 - 42*x1 - 14;
    double df_dx2 = 4*x2*x2*x2 + 4*x1*x2 + 2*x1*x1 - 26*x2 - 22;

    return Eigen::Vector2d{df_dx1, df_dx2};

}

Eigen::Matrix2d CostFunctionHimmelblau::calculateHessian(Eigen::Vector2d x) {
    double x1 = x(0);
    double x2 = x(1);

    double h11 = 12*x1*x1 + 4*x2 - 42;
    double h12 = 4*x1 + 4*x2;
    double h22 = 12*x2*x2 + 4*x1 - 26;

    return Eigen::Matrix2d{{h11, h12}, {h12, h22}};
}