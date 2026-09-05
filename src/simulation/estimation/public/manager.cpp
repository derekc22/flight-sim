#include "simulation/estimation/public/manager.hpp"

namespace estimation
{

	EstimationManagerOutput EstimationManager::step(
		const EstimationManagerInput& input)
	{
		dynamics::RigidBodyState Zt = input.Yt;

		if (linear_kalman_filter.has_value()) {
			Zt = linear_kalman_filter.value().step({.Yt = input.Yt,
													   .operating_point = input.trim_sol.operating_point,
													   .lin_sol = input.lin_sol,
													   .u_actual_t_1 = input.u_actual_t_1},
				input.dt);
		}
		if (extended_kalman_filter.has_value()) {
			Zt = extended_kalman_filter.value().step({.Yt = input.Yt,
														 .u_actual_t_1 = input.u_actual_t_1,
														 .model = input.model,
														 .conditions = input.conditions},
				input.dt);
		}

		return {.Zt = Zt};
	}

} // namespace estimation
