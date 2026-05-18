#pragma once
#include "simulation/control/shared/public.hpp"
#include "simulation/actuators/public.hpp"

namespace control {

    ControlOutput make_control_output(const actuators::ActuatorInputsVector_T<double>& u_cmd);

}
