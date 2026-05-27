#pragma once
#include "simulation/estimation/kalman/public.hpp"

namespace estimation {

    struct ExtendedKalmanFilterParameters : KalmanFilterParameters, KalmanFilterEstimatorParameters {};

    struct ExtendedKalmanFilter {
        ExtendedKalmanFilterParameters params;
        KalmanFilter policy;

        ExtendedKalmanFilter(const ExtendedKalmanFilterParameters& params);
        EstimationOutput step(const KalmanFilterEstimatorInput& estimator_input);
    };
}
