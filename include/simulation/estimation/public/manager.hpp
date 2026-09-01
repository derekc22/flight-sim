#pragma once
#include <optional>
#include "simulation/control/public/data/types.hpp"
#include "simulation/estimation/public/components/extended_kalman.hpp"
#include "simulation/estimation/public/components/linear_kalman.hpp"
#include "simulation/estimation/public/data/types.hpp"

namespace estimation {

    struct EstimationManager {
        std::optional<LinearKalmanEstimator> linear_kalman_estimator;
        std::optional<ExtendedKalmanEstimator> extended_kalman_estimator;

        EstimationManagerOutput step(const EstimationManagerInput& input);
        EstimationOutput step(const EstimatorInputs& inputs, double dt);

        EstimatorInputs build_estimator_inputs(
            const dynamics::RigidBodyState& Yt, 
            const trim::TrimSolution& trim_sol, 
            const linearization::LocalLinearization& lin_sol, 
            autodiff::AutoDiffModel& model, 
            const control::ControlOutput& u_actual_t_1,
            const operating::OperatingConditions& conditions
        );
    };

}
