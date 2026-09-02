#pragma once
#include "simulation/runner/public/data/types.hpp"

namespace runner {

    struct Linearization {
        linearization::LocalLinearization lin_sol;

        // initialize virtual linearization solution
        linearization::VirtualLocalLinearization virtual_lin_sol;
        analysis::EigenAnalysis eig_sol;

        LinearizationOutput step(const LinearizationInput& input);
    };

}
