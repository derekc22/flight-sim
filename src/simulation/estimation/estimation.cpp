#include <stdexcept>
#include "simulation/estimation/estimation.hpp"

namespace estimation {

    EstimationOutput EstimationProperties::step(const EstimationInput& estimation_input, bool trim_bool) {
        EstimationOutput out{ .state_estimate = estimation_input.zN_t };

        if (kalman_filter_estimator) {
            if (kalman_filter_estimator_type == EstimatorType::LinearKalmanFilter && !trim_bool) {
                throw std::runtime_error("estimation::EstimationProperties::step LinearKalmanFilter requires trim");
            }
            out = kalman_filter_estimator(estimation_input.kalman_filter_input);
        }

        return out;
    }
}
