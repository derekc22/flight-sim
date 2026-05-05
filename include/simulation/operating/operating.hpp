#pragma once
#include "simulation/actuators/surface.hpp"
#include "simulation/control/control.hpp"

namespace operating {

    struct FixedSurfaceActuatorInputs {
        double flap = 0.0; // [rad]
        double spoiler = 0.0; // [rad]
    };

    struct OperatingProperties {
        FixedSurfaceActuatorInputs fixed_surface_actuator_inputs{};
    };
}
