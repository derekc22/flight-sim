#pragma once
#include "simulation/control/shared/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/control/linear_quadratic/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"

namespace control {

    ControlOutput make_control_output(const actuators::ActuatorInputsVector_T<double>& u_cmd);

    dynamics::StateVector_T<double> unpack_state(const guidance::LinearQuadraticSetpoint& setpoint);

}
