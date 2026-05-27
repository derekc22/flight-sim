#pragma once
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"

namespace operating {

    struct OperatingConditions {
        atmospheric::StaticAtmosphericState static_atm_state;
        atmospheric::Wind windB{ constants::Zero3 };
    };

    struct OperatingPoint {
        dynamics::State_T<double> state;
        actuators::ActuatorInputs_T<double> input;
    };

    struct OperatingProperties {
        actuators::FixedActuatorInputs fixed_actuator_inputs{};
    };
}
