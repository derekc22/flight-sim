#include "simulation/aerodynamics/public/manager.hpp"

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

} // namespace aerodynamics
