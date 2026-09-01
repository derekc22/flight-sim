#pragma once
#include "simulation/runner/public/data/types.hpp"

namespace runner {

    struct Trim {
        trim::TrimSolution solution;

        TrimOutput step(const TrimInput& input);
    };

}
