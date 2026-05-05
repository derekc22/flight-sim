#pragma once
#include <functional>
#include "simulation/estimation/shared.hpp"
#include "simulation/estimation/kalman/estimators/ekf.hpp"
#include "simulation/estimation/kalman/estimators/lkf.hpp"

namespace estimation {

    enum class EstimatorType {
        None,
        LinearKalmanFilter,
        ExtendedKalmanFilter
    };

    using KalmanFilterEstimator = std::function<EstimationOutput(const KalmanFilterInput&)>;

    struct EstimationInput {
        dynamics::RigidBodyState zN_t;
        KalmanFilterInput kalman_filter_input;
    };

    struct EstimationProperties {
        EstimatorType kalman_filter_estimator_type = EstimatorType::None;
        KalmanFilterEstimator kalman_filter_estimator;

        EstimationOutput step(const EstimationInput& estimation_input, bool trim_bool);
    };
}
