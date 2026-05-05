#pragma once
#include "simulation/estimation/shared.hpp"

namespace estimation {

    struct ExtendedKalmanFilterParameters : EstimatorParameters {};

    struct ExtendedKalmanFilter {
        ExtendedKalmanFilter(const ExtendedKalmanFilterParameters& params);
        EstimationOutput step(const KalmanFilterInput& kalman_filter_input);
    };
}
