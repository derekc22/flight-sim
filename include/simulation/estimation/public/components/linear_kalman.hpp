#pragma once
#include <optional>
#include "simulation/estimation/public/data/types.hpp"

namespace estimation {

    struct LinearKalmanEstimator {
        LinearKalmanFilterParameters params;
        std::optional<KalmanState> state;

        LinearKalmanEstimator(const LinearKalmanFilterParameters& params);
        dynamics::RigidBodyState step(const LinearKalmanEstimatorInput& input, double dt);

        KalmanState predict(const linearization::DiscretizedLocalLinearization& lin_sol_k, const actuators::ActuatorInputsVector_T<double>& ut_1);
        KalmanState correct(const linearization::OutputJacobian& output_jacobian, const dynamics::StateVector_T<double>& yt);
    };

}
