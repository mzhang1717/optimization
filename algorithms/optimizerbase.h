#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "../objectivefunction/costfunctionbase.h"

/**
 * Abstract base class for unconstrained nonlinear optimizers.
 *
 * Minimizes a scalar cost function f(x) by iterating: compute search direction,
 * choose step size via line search, then update x. Subclasses implement
 * calculateSearchDirection() (e.g. steepest descent, Newton, BFGS); the base
 * provides the optimize() loop, Armijo backtracking line search, and termination
 * (gradient norm or max iterations). Uses Eigen::VectorXd for x, gradient, and
 * search direction; the cost function is referenced via CostFunctionBase*.
 */
class OptimizerBase {
public:
    OptimizerBase();
    OptimizerBase(CostFunctionBase& costfunction, const Eigen::Ref<const Eigen::VectorXd>& x_ini);
    ~OptimizerBase();

    /// Backtracking line search: shrink step_size_ until Armijo condition holds (or max_linesearch_ / min step).
    virtual void backtrackingLineSearch();
    /// Compute search_direction_; must be overridden (e.g. -gradient, -H^{-1}*g, or quasi-Newton direction).
    virtual void calculateSearchDirection() = 0;
    /// True if gradient norm <= gradient_epsilon_ or number_iterations >= max_iterations_.
    virtual bool isTerminationReady();
    /// Set x_ = initial_guess_, evaluate f and gradient, resize search_direction_; called at start of optimize().
    virtual void initialUpdate();
    /// Advance: x_ += step_size_ * search_direction_, then recompute f and gradient.
    virtual void update();
    /// Main entry: initialUpdate(), then loop (direction, line search, update) until termination or NaN; showResults().
    virtual void optimize();

    /// Set a new initial point for a subsequent call to optimize().
    void setInitialGuess(const Eigen::Ref<const Eigen::VectorXd>& x_ini);
    /// Print current x_, function_value_, gradient_, and number_iterations to stdout.
    void showResults() const;

    // --- State (current iterate and cost) ---
    CostFunctionBase* ptr_cost_function_;  ///< The cost function to minimize (not owned).
    double function_value_;                ///< f(x_) at the current iterate.
    Eigen::VectorXd gradient_;            ///< Gradient at x_.
    Eigen::VectorXd x_;                   ///< Current iterate.

    // --- Step and direction ---
    double step_size_;                     ///< Step length from last line search.
    Eigen::VectorXd search_direction_;    ///< Direction from last calculateSearchDirection().

    // --- Termination and iteration limits ---
    double gradient_epsilon_;              ///< Stop when ||gradient_|| <= this (set by subclass).
    int max_iterations_;                   ///< Stop after this many updates (default 5000).
    int number_iterations_;                 ///< Number of update() calls so far in this run.
    int max_linesearch_;                   ///< Max backtracking steps per iteration (default 50).
    double min_step_size_;                 ///< Lower bound for line search step before giving up.

    Eigen::VectorXd initial_guess_;       ///< Starting point for optimize().

    // --- Line search parameters (set by subclass) ---
    double initial_step_size_;             ///< First trial step in backtracking.
    double shrink_factor_;                 ///< Multiply step_size_ by this when shrinking (e.g. 0.5).
    double slope_factor_;                  ///< Armijo: sufficient decrease factor (e.g. 1e-4).
    double curve_factor_;                  ///< Used by Wolfe curvature condition in e.g. BFGS (optional).
};
