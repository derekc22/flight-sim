#pragma once
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/actuators/shared.hpp"

namespace actuators {

    struct ActuatorProperties {
        SurfaceActuators surface_actuators;
        SurfaceActuatorInputs_T<double> step(const SurfaceActuatorInputs_T<double>& u_cmd);

        PropulsorActuators propulsor_actuators;
        PropulsorActuatorInputs_T<double> step(const PropulsorActuatorInputs_T<double>& u_cmd);
    };

    ActuatorLimits_T<double> pack_actuator_limits(const SurfaceActuators& surface_actuators, const PropulsorActuators& propulsor_actuators);

}
