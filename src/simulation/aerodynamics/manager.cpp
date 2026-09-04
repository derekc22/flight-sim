#include "simulation/aerodynamics/public/manager.hpp"

#include "simulation/aerodynamics/public/detail/loads.hpp"
#include "simulation/aerodynamics/public/detail/state.hpp"

#include <utility>

namespace aerodynamics
{

	AerodynamicsManager::AerodynamicsManager(
		std::vector<Surface> s)
		: surfaces(std::move(s))
	{
		compute_surface_geometry();
	}

	void AerodynamicsManager::compute_surface_geometry()
	{
		for (Surface& s : surfaces) {
			s.area = s.chord * s.span;
			s.AR = s.span / s.chord;
		}
	}

	AerodynamicsManagerOutput AerodynamicsManager::step(
		const AerodynamicsManagerInput& input)
	{
		return {
			.aero_t = compute_aerodynamic_state(input.X, input.windB),
			.WB_aerodynamic = step_aero_forces_moments(surfaces, input.pB_GB, input.X, input.atm, input.u, input.windB)
		};
	}

} // namespace aerodynamics
