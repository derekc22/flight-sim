#pragma once
#include "simulation/actuators/surface.hpp"
#include "simulation/actuators/propulsor.hpp"
#include "simulation/control/shared.hpp"
#include "simulation/types/types.hpp"

namespace actuators {

    struct ActuatorProperties {
        SurfaceActuators surface_actuators;
        types::SurfaceActuatorInputs_T<double> step(const types::SurfaceActuatorInputs_T<double>& u_cmd);

        PropulsorActuators propulsor_actuators;
        types::PropulsorActuatorInputs_T<double> step(const types::PropulsorActuatorInputs_T<double>& u_cmd);
    };

}
