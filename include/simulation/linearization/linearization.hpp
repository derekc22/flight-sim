#pragma once
#include <string>
#include "simulation/linearization/shared.hpp"
#include "simulation/trim/shared.hpp"

namespace vehicles { struct Aircraft; } // forward declare

namespace linearization {
    TrimLinearization linearize_trim_solution(vehicles::Aircraft& aircraft, const trim::TrimSolution& trim_sol);

    std::string print_linearization_solution(const TrimLinearization& lin);

    DiscretizedTrimLinearization discretize(const linearization::TrimLinearization& lin_sol);
}
