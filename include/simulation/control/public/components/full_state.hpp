#pragma once
#include "simulation/control/public/data/types.hpp"

namespace control {

    struct FullStateControl {
        FullStateControlImplementation implementation;

        FullStateControl(const LinearQuadraticRegulatorParameters& params);
        FullStateControl(const LinearQuadraticIntegratorParameters& params);
        ControlComponentOutput step(const FullStateControlInput& input, double dt);
    };

}
