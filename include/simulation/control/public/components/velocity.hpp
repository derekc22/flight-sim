#pragma once
#include "simulation/control/public/data/types.hpp"

namespace control {

    struct VelocityControl {
        ControllerType controller_type;
        VelocityControlImplementation implementation;

        VelocityControl(ControllerType controller_type, const VelocityPIDParameters& params);
        ControlComponentOutput step(const VelocityControlInput& input, double dt);
    };

}
