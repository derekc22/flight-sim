#include "simulation/estimation/public.hpp"
#include "simulation/control/shared/public.hpp"

namespace estimation {

    EstimationOutput EstimationProperties::step(const EstimatorInputs& inputs, double dt) {
        EstimationOutput out{ .Zt = inputs.Yt };

        if (linear_kalman_estimator && inputs.linear_kalman_estimator_input.has_value()) {
            out = linear_kalman_estimator(inputs.linear_kalman_estimator_input.value(), dt);
        }
        if (extended_kalman_estimator && inputs.extended_kalman_estimator_input.has_value()) {
            out = extended_kalman_estimator(inputs.extended_kalman_estimator_input.value(), dt);
        }

        return out;
    }

    EstimatorInputs EstimationProperties::build_estimator_inputs(
        const dynamics::RigidBodyState& Yt, 
        const trim::TrimSolution& trim_sol, 
        const linearization::LocalLinearization& lin_sol, 
        autodiff::AutoDiffModel& model, 
        const control::ControlOutput& u_actual_t_1,
        const operating::OperatingConditions& conditions
    ) {
        EstimatorInputs estimator_inputs{ .Yt = Yt };

        if (linear_kalman_estimator_type != EstimatorType::None) {
            estimator_inputs.linear_kalman_estimator_input.emplace(LinearKalmanEstimatorInput{
                .Yt = Yt,
                .operating_point = trim_sol.operating_point,
                .lin_sol = lin_sol,
                .u_actual_t_1 = u_actual_t_1
            });
        }
        if (extended_kalman_estimator_type != EstimatorType::None) {
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
