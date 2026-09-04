#pragma once
#include "simulation/aerodynamics/public/detail/loads.hpp"

namespace aerodynamics
{

	template <typename T>
	AerodynamicsManagerOutput_T<T> AerodynamicsManager::step(const AerodynamicsManagerInput_T<T>& input)
	{
		return {
			.WB_aerodynamic =
				step_aero_forces_moments_T<T>(surfaces, input.pB_GB, input.twist, input.atm, input.u, input.windB)
		};
	}

} // namespace aerodynamics
