#pragma once
#include "simulation/allocator/private/data/types.hpp"
#include "simulation/allocator/public/data/types.hpp"

#include <tuple>

namespace allocator
{

	std::tuple<EffectivenessMatrix, dynamics::WrenchVector_T<double>> compute_effectiveness_matrix(
		const autodiff::AutoDiffModel& model,
		const operating::OperatingPoint_T<double>& operating_point,
		const operating::OperatingConditions& conditions);

}
