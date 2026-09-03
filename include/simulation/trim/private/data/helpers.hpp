#pragma once
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/trim/private/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace trim {

    template <typename T>
    TrimResidual_T<T> pack_trim_residual_T(const TrimResidualVector_T<T>& residual);

    TrimResidualVector_T<double> fetch_trim_residual_weights(const TrimSolveOptions& options);

    TrimSolution build_trim_solution(const operating::StateInputVector_T<double>& xu, const TrimResidualVector_T<double>& residual, const TrimResidualVector_T<double>& weighted_residual, const autodiff::AutoDiffModel& model, const operating::OperatingConditions& conditions, bool converged, std::size_t iterations);

}

#include "simulation/trim/private/data/helpers.tpp"
