#include "simulation/aerodynamics/public/manager.hpp"

#include <utility>

namespace aerodynamics
{

	AerodynamicsManager::AerodynamicsManager(
		std::vector<Surface> s)
		: surfaces(std::move(s))
	{
		for (Surface& surface : surfaces) {
			surface.compute_geometry();
		}
	}

} // namespace aerodynamics
