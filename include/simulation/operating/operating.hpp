#pragma once
#include "simulation/actuators/surface.hpp"
#include "simulation/control/control.hpp"

namespace operating {

    struct FixedControls {
        double flap = 0.0;
        double spoiler = 0.0;
    };

    struct OperatingProperties {
        FixedControls fixed_controls{};
    };

    control::SurfaceActuatorInputs fixed_surface_actuator_inputs(const OperatingProperties& operating_properties);
}
