#pragma once
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/trim/private/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace trim {

    void validate_trim_solve_options(const TrimSolveOptions& options);

    TrimSolution solve_trim(const TrimProblem& problem, const autodiff::AutoDiffModel& model, TrimSolveOptions options = {});

}
