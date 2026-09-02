#pragma once
#include "simulation/runner/public/data/types.hpp"

namespace runner {

    struct Trim {
        trim::TrimSolution trim_sol;

        TrimOutput step(const TrimInput& input);
    };

}
