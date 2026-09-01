#include "simulation/estimation/public/manager.hpp"

namespace estimation {

    EstimationManagerOutput EstimationManager::step(const EstimationManagerInput& input) {
        const EstimatorInputs estimator_inputs = build_estimator_inputs(
            input.measured_state,
            input.trim_solution,
            input.linearization,
            input.model,
            input.previous_actual_inputs,
            input.conditions
        );
        return { .estimated_state = step(estimator_inputs, input.dt).Zt };
    }

    EstimationOutput EstimationManager::step(const EstimatorInputs& inputs, double dt) {
        EstimationOutput out{ .Zt = inputs.Yt };

        if (linear_kalman_estimator.has_value() && inputs.linear_kalman_estimator_input.has_value()) {
            out = linear_kalman_estimator.value().step(inputs.linear_kalman_estimator_input.value(), dt);
        }
        if (extended_kalman_estimator.has_value() && inputs.extended_kalman_estimator_input.has_value()) {
            out = extended_kalman_estimator.value().step(inputs.extended_kalman_estimator_input.value(), dt);
        }

        return out;
    }

    EstimatorInputs EstimationManager::build_estimator_inputs(
        const dynamics::RigidBodyState& Yt, 
        const trim::TrimSolution& trim_sol, 
        const linearization::LocalLinearization& lin_sol, 
        autodiff::AutoDiffModel& model, 
        const control::ControlOutput& u_actual_t_1,
        const operating::OperatingConditions& conditions
    ) {
        EstimatorInputs estimator_inputs{ .Yt = Yt };

        if (linear_kalman_estimator.has_value()) {
            estimator_inputs.linear_kalman_estimator_input.emplace(LinearKalmanEstimatorInput{
                .Yt = Yt,
                .operating_point = trim_sol.operating_point,
                .lin_sol = lin_sol,
                .u_actual_t_1 = u_actual_t_1
            });
        }
        if (extended_kalman_estimator.has_value()) {
            estimator_inputs.extended_kalman_estimator_input.emplace(ExtendedKalmanEstimatorInput{
                .Yt = Yt,
                .u_actual_t_1 = u_actual_t_1,
                .model = model,
                .conditions = conditions
            });
        }

        return estimator_inputs;
    }

}
