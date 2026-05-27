#pragma once
#include "simulation/estimation/public.hpp"

namespace json {

    template <typename EstimatorType, typename EstimatorClass, typename EstimatorParametersType>
    EstimatorClass make_stateful_kalman_filter_estimator(const EstimatorParametersType& params) {
        return [estimator = EstimatorType{ params }](const estimation::KalmanFilterEstimatorInput& input) mutable {
            return estimator.step(input);
        };
    }
}
