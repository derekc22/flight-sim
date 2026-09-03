#pragma once
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/linearization/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"

namespace linearization {

	LocalLinearization linearize_operating_point(const autodiff::AutoDiffModel& model,
	    const operating::OperatingPoint_T<double>& operating_point,
	    const operating::OperatingConditions& conditions);

	VirtualLocalLinearization linearize_virtual_operating_point(const autodiff::AutoDiffModel& model,
	    const operating::VirtualOperatingPoint_T<double>& virtual_operating_point);

} // namespace linearization
