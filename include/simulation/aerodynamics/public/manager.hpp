#pragma once
#include "simulation/aerodynamics/public/components/surface.hpp"

#include <vector>

namespace aerodynamics
{

	struct AerodynamicsManager {
		std::vector<Surface> surfaces;

		AerodynamicsManager(std::vector<Surface> s);

		/**
		 * @brief Computes the total aerodynamic wrench for one simulation step.
		 *
		 * @tparam T Scalar type used for the computation.
		 * @param[in] input Body state, atmosphere, actuator inputs, and wind used for the calculation.
		 * @return Total aerodynamic force [N] and moment [N m] about the center of gravity.
		 */
		template <typename T> AerodynamicsManagerOutput_T<T> step(const AerodynamicsManagerInput_T<T>& input);
	};

} // namespace aerodynamics

#include "simulation/aerodynamics/public/manager.tpp"
