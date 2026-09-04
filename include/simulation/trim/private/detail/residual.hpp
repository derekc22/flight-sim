#pragma once
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/trim/private/data/types.hpp"

namespace trim
{

	template <typename T>
	TrimResidualVector_T<T> compute_trim_residual_vector_T(const operating::StateInputVector_T<T>& xu,
		const autodiff::AutoDiffModel& model,
		const TrimTarget& target,
		const operating::OperatingConditions& conditions);

	double residual_norm_inf(const TrimResidualVector_T<double>& residual);

	TrimResidualJacobian compute_trim_residual_jac(const operating::StateInputVector_T<double>& xu,
		const autodiff::AutoDiffModel& model,
		const TrimTarget& target,
		const operating::OperatingConditions& conditions);

} // namespace trim

#include "simulation/trim/private/detail/residual.tpp"
