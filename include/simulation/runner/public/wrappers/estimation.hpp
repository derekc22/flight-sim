#pragma once
#include "simulation/runner/public/data/types.hpp"

namespace runner {

    struct EstimationWrapper {
        dynamics::RigidBodyState Zt_1;

        EstimationWrapperOutput step(const EstimationWrapperInput& input);
    };

}
