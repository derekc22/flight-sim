#pragma once
#include "simulation/control/public/data/types.hpp"

namespace control {

    struct FullStateControl {
        ControllerType controller_type;
        FullStateControlImplementation implementation;

        FullStateControl(ControllerType controller_type, const LinearQuadraticRegulatorParameters& params);
        FullStateControl(ControllerType controller_type, const LinearQuadraticIntegratorParameters& params);
        ControlComponentOutput step(const FullStateControlInput& input, double dt);
    };

}
