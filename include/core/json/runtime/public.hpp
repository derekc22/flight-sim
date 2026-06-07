#pragma once
#include "simulation/actuators/public.hpp"
#include "simulation/runtime/public.hpp"

namespace json {

    runtime::RuntimeProperties parse_runtime_config(const actuators::ActuatorProperties& actuator_properties);

}
