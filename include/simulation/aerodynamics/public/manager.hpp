#pragma once
#include "simulation/aerodynamics/public/components/surface.hpp"

#include <vector>

namespace aerodynamics
{

	struct AerodynamicsManager {
		std::vector<Surface> surfaces;

		AerodynamicsManager(std::vector<Surface> s);
		template <typename T> AerodynamicsManagerOutput_T<T> step(const AerodynamicsManagerInput_T<T>& input);
	};

} // namespace aerodynamics

#include "simulation/aerodynamics/public/manager.tpp"
