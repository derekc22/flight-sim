#pragma once
#include "simulation/estimation/shared.hpp"

namespace estimation {

    struct ExtendedKalmanFilterParameters : KalmanFilterEstimatorParameters {};

    struct ExtendedKalmanFilter {
        ExtendedKalmanFilter(const ExtendedKalmanFilterParameters& params);
        EstimationOutput step(const KalmanFilterInput& estimator_input);
    };
}
