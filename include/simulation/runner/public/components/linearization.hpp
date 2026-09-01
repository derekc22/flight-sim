#pragma once
#include "simulation/runner/public/data/types.hpp"

namespace runner {

    struct Linearization {
        linearization::LocalLinearization physical;

        // initialize virtual linearization solution
        linearization::VirtualLocalLinearization virtual_model;
        analysis::EigenAnalysis eigenanalysis;

        LinearizationOutput step(const LinearizationInput& input);
    };

}
