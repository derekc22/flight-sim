#pragma once
#include "simulation/actuators/public.hpp"
#include "simulation/settings/public.hpp"

namespace json {

    settings::SettingsManager parse_settings_config(const actuators::ActuatorManager& actuator_manager);

}
