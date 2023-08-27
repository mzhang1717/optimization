#include "costfunctionexample.h"
#include <cmath>

double CostFunctionExample::calculateCostFunctionValue(double x) {
    
    return pow(x,8)-27*pow(x,7)+288*pow(x,6)-1512*pow(x,5)+3888*pow(x,4)-3888*pow(x,3);
}

double CostFunctionExample::calculateGradient(double x) {

    return 8*pow(x,7)-7*27*pow(x,6)+6*288*pow(x,5)-5*1512*pow(x,4)+4*3888*pow(x,3)-3*3888*pow(x,2);

}

double CostFunctionExample::calculateHessian(double x) {
    return 56*pow(x,6)-42*27*pow(x,5)+30*288*pow(x,4)-20*1512*pow(x,3)+12*3888*pow(x,2)-6*3888*x;

}