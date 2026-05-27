#pragma once
#include <functional>
#include "simulation/estimation/shared/public.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/trim/public.hpp"

namespace estimation {

    enum class EstimatorType {
        None,
        LinearKalmanFilter,
        ExtendedKalmanFilter
    };

    using KalmanFilterEstimator = std::function<EstimationOutput(const KalmanFilterEstimatorInput&)>;

    struct EstimationInput {
        const dynamics::RigidBodyState& yN_t;
        const KalmanFilterEstimatorInput& estimator_input;
    };

    struct EstimationProperties {
        EstimatorType kalman_filter_estimator_type = EstimatorType::None;
        KalmanFilterEstimator kalman_filter_estimator;

        EstimationOutput step(const EstimationInput& estimation_input, bool trim_bool);
    };
}
