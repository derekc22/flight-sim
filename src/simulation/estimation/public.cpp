#include <stdexcept>
#include "simulation/estimation/public.hpp"
#include "simulation/control/shared/public.hpp"

namespace estimation {

    EstimationOutput EstimationProperties::step(const EstimatorInputs& inputs, double dt, bool trim_flag) {
        EstimationOutput out{ .Zt = inputs.Yt };

        if (extended_kalman_estimator) {
            out = extended_kalman_estimator(inputs.extended_kalman_estimator_input, dt);
        }

        if (!trim_flag) {
            if (linear_kalman_estimator) { 
                throw std::runtime_error("estimation::EstimationProperties::step LinearKalmanEstimator requires trim"); 
            }
        }

        if (trim_flag) {
            if (linear_kalman_estimator) {
                out = linear_kalman_estimator(inputs.linear_kalman_estimator_input, dt);
            }
        }

        return out;
    }

    EstimatorInputs build_estimator_inputs(const dynamics::RigidBodyState& Yt, const trim::TrimSolution& trim_sol, const linearization::LocalLinearization& lin_sol, autodiff::AutoDiffModel& model, const control::ControlOutput& u_cmd_t_1, const operating::OperatingConditions& conditions) {

        EstimatorInputs estimator_inputs {
            .Yt = Yt,
            .linear_kalman_estimator_input = LinearKalmanEstimatorInput {
                .Yt = Yt,
                .operating_point = trim_sol.operating_point,
                .lin_sol = lin_sol,
                .u_cmd_t_1 = u_cmd_t_1
            },
            .extended_kalman_estimator_input = ExtendedKalmanEstimatorInput {
                .Yt = Yt,
                .u_cmd_t_1 = u_cmd_t_1,
                .model = model,
                .conditions = conditions
            }
        };

        return estimator_inputs;
    }
}
