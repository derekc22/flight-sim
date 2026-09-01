#pragma once
#include <optional>
#include "simulation/estimation/public/data/types.hpp"

namespace estimation {

    struct LinearKalmanEstimator {
        LinearKalmanFilterParameters params;
        std::optional<KalmanState> state;

        LinearKalmanEstimator(const LinearKalmanFilterParameters& params);
        EstimationOutput step(const LinearKalmanEstimatorInput& input, double dt);

        KalmanState predict(const linearization::DiscretizedLocalLinearization& lin_sol, const actuators::ActuatorInputsVector_T<double>& previous_actual_inputs);
        KalmanState correct(const linearization::OutputJacobian& output_jacobian, const dynamics::StateVector_T<double>& measured_state);
    };

}
