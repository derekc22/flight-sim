#pragma once
#include "simulation/actuators/public.hpp"

namespace operating {

    struct OperatingProperties {
        actuators::FixedActuatorInputs_T fixed_actuator_inputs{};
    };
}
