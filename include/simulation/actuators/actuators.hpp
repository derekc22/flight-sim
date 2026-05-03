#pragma once
#include "simulation/actuators/surface.hpp"
#include "simulation/actuators/propulsor.hpp"
#include "simulation/control/shared.hpp"

namespace actuators {

    struct ActuatorProperties {
        SurfaceActuators surface_actuators;
        control::SurfaceActuatorInputs step(const control::SurfaceActuatorInputs& u_cmd);

        PropulsorActuators propulsor_actuators;
        control::PropulsorActuatorInputs step(const control::PropulsorActuatorInputs& u_cmd);
    };

}
