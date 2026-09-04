#pragma once
#include "simulation/aerodynamics/public/data/types.hpp"

#include <vector>

namespace aerodynamics
{

	struct AerodynamicsManager {
		std::vector<Surface> surfaces;

		AerodynamicsManager(std::vector<Surface> s);
		void compute_surface_geometry();
		AerodynamicsManagerOutput step(const AerodynamicsManagerInput& input);
	};

} // namespace aerodynamics
