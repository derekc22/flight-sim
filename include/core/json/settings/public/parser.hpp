#pragma once
#include "simulation/actuators/public/manager.hpp"
#include "simulation/settings/public/data/types.hpp"

namespace json {

    settings::Settings parse_settings_config(const actuators::ActuatorManager& actuator_manager);

}
