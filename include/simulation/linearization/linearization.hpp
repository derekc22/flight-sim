#pragma once
#include <iostream>
#include "simulation/trim/trim.hpp"
#include "simulation/linearization/types.hpp"

namespace linearization {
    TrimLinearization linearize_trim_solution(vehicles::Aircraft& aircraft, const trim::TrimSolution& trim_sol);

    std::string print_linearization_solution(const TrimLinearization& lin);

    DiscretizedTrimLinearization discretize(const linearization::TrimLinearization& lin_sol);
}
