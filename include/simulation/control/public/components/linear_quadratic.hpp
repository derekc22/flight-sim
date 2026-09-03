#pragma once
#include "simulation/control/public/data/types.hpp"

namespace control {

    struct LinearQuadraticControl {
        LinearQuadraticControlImplementation implementation;

        LinearQuadraticControl(const LinearQuadraticRegulatorParameters& params);
        LinearQuadraticControl(const LinearQuadraticIntegratorParameters& params);
        ControlComponentOutput step(const LinearQuadraticControlInput& input, double dt);
    };

}
