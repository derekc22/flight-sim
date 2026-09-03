#include "simulation/constants/public/scalars.hpp"
#include "simulation/estimation/public/manager.hpp"
#include "simulation/runner/public/wrappers/estimation.hpp"
#include "simulation/runner/public/scheduling/scheduler.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace runner {

	EstimationWrapperOutput EstimationWrapper::step(
	    const EstimationWrapperInput& input)
	{
		estimation::EstimationManager& estimation_manager = input.aircraft.estimation_manager;

		// initialize estimated state to measurements
		dynamics::RigidBodyState Zt = input.context.Yt;

		if (input.enabled) {
			if (input.scheduler.estimation_tick >= constants::hz) {
				double estimation_dt = input.scheduler.estimation_elapsed_ticks * constants::dt;

				// overwrite local estimated state with estimator result
				Zt = estimation_manager
				         .step({.Yt = input.context.Yt,
				             .trim_sol = input.trim_sol,
				             .lin_sol = input.lin_sol,
				             .model = input.context.autodiff_model,
				             .u_actual_t_1 = input.u_actual_t_1,
				             .conditions = input.context.transient_conditions,
				             .dt = estimation_dt})
				         .Zt;
				Zt_1 = Zt;

				input.scheduler.estimation_tick -= constants::hz;
				input.scheduler.estimation_elapsed_ticks = 0;
			} else
				Zt = Zt_1; // perform ZOH
		}

		return {.Zt = Zt};
	}

} // namespace runner
