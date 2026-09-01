#pragma once
#include "simulation/runner/public/data/types.hpp"

namespace runner {

    struct Estimation {
        dynamics::RigidBodyState Zt_1;

        EstimationOutput step(const EstimationInput& input);
    };

}
