#pragma once
#include "simulation/aerodynamics/public/manager.hpp"

namespace aerodynamics
{

	template <typename T>
	AerodynamicsManagerOutput_T<T> AerodynamicsManager::step(
		const AerodynamicsManagerInput_T<T>& input)
	{
		const SurfaceInput_T<T> surface_input{
			.pB_GB = input.pB_GB, .twist = input.twist, .atm = input.atm, .u = input.u, .windB = input.windB
		};

		dynamics::Wrench_T<T> WB_aerodynamic;
		for (Surface& surface : surfaces) {
			const dynamics::Wrench_T<T> WB_surface = surface.step<T>(surface_input);
			WB_aerodynamic.F += WB_surface.F;
			WB_aerodynamic.M += WB_surface.M;
		}

		return {.WB_aerodynamic = WB_aerodynamic};
	}

} // namespace aerodynamics
