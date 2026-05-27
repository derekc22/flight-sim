#pragma once
#include "simulation/estimation/kalman/public.hpp"

namespace estimation {

    struct LinearKalmanFilterParameters : KalmanFilterParameters, KalmanFilterEstimatorParameters {};

    struct LinearKalmanFilter {
        LinearKalmanFilterParameters params;
        KalmanFilter policy;

        LinearKalmanFilter(const LinearKalmanFilterParameters& params);
        EstimationOutput step(const KalmanFilterEstimatorInput& estimator_input);
    };
}
