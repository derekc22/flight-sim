#pragma once
#include "simulation/propulsion/public.hpp"

namespace propulsion {

    double step_propeller_omega_dot(actuators::PropulsorActuator& propulsor, double thrust, const atmospheric::StaticAtmosphericState& static_atm_state);

}
