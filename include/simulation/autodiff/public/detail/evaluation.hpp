#pragma once
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"

namespace autodiff {

	template <typename T>
	dynamics::Wrench_T<T> compute_net_wrench_T(const operating::OperatingPoint_T<T>& operating_point,
	    const AutoDiffModel& model,
	    const operating::OperatingConditions& conditions,
	    T dt);

	template <typename T>
	dynamics::StateDot_T<T> compute_state_dot_T(const operating::OperatingPoint_T<T>& operating_point,
	    const AutoDiffModel& model,
	    const operating::OperatingConditions& conditions,
	    T dt);

	template <typename T>
	dynamics::StateDot_T<T> compute_state_dot_T(const operating::VirtualOperatingPoint_T<T>& operating_point,
	    const AutoDiffModel& model);

	template <typename T>
	dynamics::StateDot_T<T> compute_state_dot_from_net_wrench_T(const dynamics::State_T<T>& x,
	    const AutoDiffModel& model,
	    const dynamics::Wrench_T<T>& net_wrench);

} // namespace autodiff

#include "simulation/autodiff/public/detail/evaluation.tpp"
