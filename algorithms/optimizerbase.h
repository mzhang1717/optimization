#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "../objectivefunction/costfunctionbase.h"

class OptimizerBase {
public:
    OptimizerBase();
    OptimizerBase(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini);
    ~OptimizerBase();

    virtual void backtrackingLineSearch();
    virtual void calculateSearchDirection();
    virtual bool isTerminationReady();
    virtual void initialUpdate();
    virtual void update();
    virtual void optimize();
    void setInitialGuess(const Eigen::Ref<const Eigen::VectorXd>& x_ini);

    void showResults();


    CostFunctionBase* ptr_cost_function_;
    double function_value_;
    Eigen::VectorXd gradient_;
    Eigen::VectorXd x_;
 
    double step_size_;
    Eigen::VectorXd search_direction_;

    double gradient_epsilon_;
    int max_iterations_;
    int number_iterations;
    
    Eigen::VectorXd initial_guess_;

    //line search parameters
    double initial_step_size_;
    double shrink_factor_;
    double slope_factor_;
};