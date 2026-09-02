#pragma once
#include "simulation/constants/public.hpp"
#include "simulation/runner/public/data/types.hpp"

namespace runner {

    struct Physics {
        // initialize prior-step net wrench
        dynamics::Wrench WB_net_t_1{
            .F = dynamics::Force { constants::Zero3 }, 
            .M = dynamics::Moment{ constants::Zero3 } 
        };

        PhysicsOutput step(const PhysicsInput& input);
    };

}
