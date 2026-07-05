#pragma once
#include <functional>
#include "simulation/estimation/shared/public.hpp"
#include "simulation/dynamics/public.hpp"

namespace estimation {

    enum class EstimatorType {
        None,
        LinearKalmanFilter,
        ExtendedKalmanFilter
    };

    using LinearKalmanEstimator = std::function<EstimationOutput(const LinearKalmanEstimatorInput&, double dt)>;

    using ExtendedKalmanEstimator = std::function<EstimationOutput(const ExtendedKalmanEstimatorInput&, double dt)>;

    struct EstimatorInputs {
        const dynamics::RigidBodyState& Yt;
        const LinearKalmanEstimatorInput& linear_kalman_estimator_input;
        const ExtendedKalmanEstimatorInput& extended_kalman_estimator_input;
    };

    struct EstimationProperties {
        EstimatorType linear_kalman_estimator_type = EstimatorType::None;
        EstimatorType extended_kalman_estimator_type = EstimatorType::None;

        LinearKalmanEstimator linear_kalman_estimator;
        ExtendedKalmanEstimator extended_kalman_estimator;

        EstimationOutput step(const EstimatorInputs& inputs, double dt, bool trim_flag);
    };
}
