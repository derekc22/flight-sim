#pragma once
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/guidance/public/data/types.hpp"

namespace control
{

	dynamics::StateVector_T<double> unpack_state(const guidance::LinearQuadraticSetpoint& setpoint);

}
