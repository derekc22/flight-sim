#include <stdexcept>
#include "simulation/estimation/public.hpp"

namespace estimation {

    EstimationOutput EstimationProperties::step(const EstimatorInputs& inputs, bool trim_flag) {
        EstimationOutput out{ .Zt = inputs.Yt };

        if (extended_kalman_estimator) {
            out = extended_kalman_estimator(inputs.extended_kalman_estimator_input);
        }

        if (!trim_flag) {
            if (linear_kalman_estimator) { 
                throw std::runtime_error("estimation::EstimationProperties::step LinearKalmanEstimator requires trim"); 
            }
        }

        if (trim_flag) {
            if (linear_kalman_estimator) {
                out = linear_kalman_estimator(inputs.linear_kalman_estimator_input);
            }
        }

        return out;
    }
}
