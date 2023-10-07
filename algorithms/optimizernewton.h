#include <cmath>
#include "optimizerbase.h"

class OptimizerNewton : public OptimizerBase {
public:
    OptimizerNewton() {}
    ~OptimizerNewton() {}

    void CalculateSearchDirection();
    void CalculateStepSize();
    bool CheckTerminateCriteria();
    void Update();
private:
    double cost_hessian_;
};