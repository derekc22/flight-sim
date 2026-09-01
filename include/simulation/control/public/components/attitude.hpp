#pragma once
#include "simulation/control/public/data/types.hpp"

namespace control {

    struct AttitudeControl {
        ControllerType controller_type;
        AttitudeControlImplementation implementation;

        AttitudeControl(ControllerType controller_type, const AttitudePIDParameters& params);
        ControlComponentOutput step(const AttitudeControlInput& input, double dt);
    };

}
