#pragma once
#include "simulation/actuators/surface.hpp"
#include "simulation/control/control.hpp"
#include "simulation/types/types.hpp"

namespace operating {

    struct OperatingProperties {
        types::FixedActuatorInputs_T<double> fixed_actuator_inputs{};
    };
}
