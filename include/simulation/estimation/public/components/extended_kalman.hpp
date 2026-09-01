#pragma once
#include <optional>
#include <tuple>
#include "simulation/estimation/public/data/types.hpp"

namespace estimation {

    struct ExtendedKalmanEstimator {
        ExtendedKalmanFilterParameters params;
        std::optional<KalmanState> state;

        ExtendedKalmanEstimator(const ExtendedKalmanFilterParameters& params);
        EstimationOutput step(const ExtendedKalmanEstimatorInput& input, double dt);

        std::tuple<KalmanState, linearization::OutputJacobian> predict(const ExtendedKalmanEstimatorInput& input, const actuators::ActuatorInputsVector_T<double>& previous_actual_inputs, double dt);
        KalmanState correct(const dynamics::StateVector_T<double>& measured_state, const linearization::OutputJacobian& output_jacobian);
    };

}
