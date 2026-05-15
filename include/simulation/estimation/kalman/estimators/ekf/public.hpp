#pragma once
#include "simulation/estimation/public.hpp"

namespace estimation {

    struct ExtendedKalmanFilterParameters : KalmanFilterEstimatorParameters {};

    struct ExtendedKalmanFilter {
        ExtendedKalmanFilter(const ExtendedKalmanFilterParameters& params);
        EstimationOutput step(const KalmanFilterInput& estimator_input);
    };
}
