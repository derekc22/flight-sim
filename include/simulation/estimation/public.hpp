#pragma once
#include <functional>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/trim/public.hpp"

namespace estimation {

    struct EstimationOutput {
        dynamics::RigidBodyState zN_t;  // state estimate
    };

    struct KalmanFilterInput {
        dynamics::RigidBodyState yN_t;
        linearization::TrimLinearization lin_sol;
        trim::TrimSolution trim_sol;
        actuators::SurfaceActuatorInputs_T<double> u_surface_actual_prev;
        actuators::PropulsorActuatorInputs_T<double> u_propulsor_actual_prev;
    };

    struct KalmanFilterEstimatorParameters {};

    enum class EstimatorType {
        None,
        LinearKalmanFilter,
        ExtendedKalmanFilter
    };

    using KalmanFilterEstimator = std::function<EstimationOutput(const KalmanFilterInput&)>;

    struct EstimationInput {
        const dynamics::RigidBodyState& yN_t;
        const KalmanFilterInput& estimator_input;
    };

    struct EstimationProperties {
        EstimatorType kalman_filter_estimator_type = EstimatorType::None;
        KalmanFilterEstimator kalman_filter_estimator;

        EstimationOutput step(const EstimationInput& estimation_input, bool trim_bool);
    };
}
