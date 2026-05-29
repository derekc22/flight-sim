#pragma once
#include "simulation/actuators/public.hpp"

namespace operating {

    struct OperatingProperties {
        actuators::FixedActuatorInputs fixed_actuator_inputs{};
    };
}
