#pragma once
#include "simulation/allocator/private/data/types.hpp"
#include "simulation/allocator/public/data/types.hpp"

#include <tuple>

namespace allocator
{

	/**
	 * @brief Computes the local actuator-effectiveness matrix and net wrench.
	 *
	 * @param[in] model Differentiable aircraft model and actuator configuration.
	 * @param[in] operating_point State and actuator input at which to evaluate the model.
	 * @param[in] conditions Atmospheric and wind conditions used by the model.
	 * @return Effectiveness matrix and body-frame net-wrench vector at the operating point.
	 */
	std::tuple<EffectivenessMatrix, dynamics::WrenchVector_T<double>> compute_effectiveness_matrix(
		const autodiff::AutoDiffModel& model,
		const operating::OperatingPoint_T<double>& operating_point,
		const operating::OperatingConditions& conditions);

} // namespace allocator
