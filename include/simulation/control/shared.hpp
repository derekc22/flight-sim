#pragma once
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"

namespace control {

    struct ControlOutput {
        actuators::SurfaceActuatorInputs_T<double> surface_inputs;
        actuators::PropulsorActuatorInputs_T<double> propulsor_inputs;
    };

}
