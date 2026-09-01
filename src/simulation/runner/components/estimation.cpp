#include "simulation/constants/public.hpp"
#include "simulation/estimation/public/manager.hpp"
#include "simulation/runner/public/components/estimation.hpp"
#include "simulation/runner/public/components/scheduler.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace runner {

    EstimationOutput Estimation::step(const EstimationInput& input) {
        estimation::EstimationManager& estimation_manager = input.aircraft.estimation_manager;

        // initialize estimated state to measurements
        dynamics::RigidBodyState Zt = input.context.Yt;

        if (input.enabled) {
            if (input.scheduler.estimation_tick >= constants::hz) {
                double estimation_dt = input.scheduler.estimation_elapsed_ticks * constants::dt;

                estimation::EstimatorInputs estimator_inputs = estimation_manager.build_estimator_inputs(
                    input.context.Yt,
                    input.trim_solution, input.linearization,
                    input.context.autodiff_model,
                    input.actual_inputs,
                    input.context.transient_conditions
                );

                // overwrite local estimated state with estimator result
                Zt = estimation_manager.step(estimator_inputs, estimation_dt).Zt;
                Zt_1 = Zt;

                input.scheduler.estimation_tick -= constants::hz;
                input.scheduler.estimation_elapsed_ticks = 0;
            }
            else Zt = Zt_1; // perform ZOH
        }

        return { .estimated_state = Zt };
    }

}
